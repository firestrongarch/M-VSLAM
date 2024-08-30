#include "feature.h"
#include "frontend.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "g2o_types.hpp"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <memory>
#include <vector>

int Frontend::OpticalFlow(LkInfo info)
{
    std::vector<cv::Point2f> prev_points, next_points;
    for(const auto feature : info.prev_features){
        prev_points.push_back(feature->pt);
    }
    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        info.prev_img,
        info.next_img,
        prev_points,
        next_points,
        status,
        error
    );

    size_t i = 0;
    std::erase_if(info.prev_features,[&](auto feature){
        if (status.at(i)){
            Feature::Ptr next_feature = std::make_shared<Feature>();
            next_feature->pt = next_points.at(i);
            next_feature->map_point_ = feature->map_point_;

            info.next_features.emplace_back(next_feature);
        }else{
            feature->is_outlier_ = true;
        }
        i++;
        return feature->is_outlier_;
    });

    return info.next_features.size();
}

Sophus::SE3d Frontend::Optimize(OptimizeInfo info)
{
    g2o::SparseOptimizer optimizer;
    using BlockSolver = g2o::BlockSolver_6_3;
    using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
    auto solver = g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>());
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
    optimizer.setAlgorithm(algorithm);

    auto vertex_pose = new VertexPose;
    vertex_pose->setId(0);

    vertex_pose->setEstimate(info.pose);
    optimizer.addVertex(vertex_pose);

    // edges
    int index = 1;
    std::vector<EdgePose *> edges;
    for(auto feature:info.features){
        std::shared_ptr<MapPoint> map_point = feature->map_point_.lock();
        if(!map_point){
            std::cout<<"map point is outlier"<<std::endl;   
            continue;
        }
        auto pt = feature->pt;
        auto edge = new EdgePose(map_point->Pos(),info.K);
        edge->setId(index);
        edge->setVertex(0,vertex_pose);
        edge->setMeasurement(Eigen::Vector2d(pt.x, pt.y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber);
        edges.emplace_back(edge);
        optimizer.addEdge(edge);
        index++;
    }

    // estimate the Pose and determine the outliers
    // start optimization
    const double chi2_th = 5.991;
    int cnt_outliers = 0;
    int num_iterations = 4;
    for (int iteration = 0; iteration < num_iterations; iteration++){
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outliers = 0;

        for (size_t i = 0, N = edges.size(); i < N; i++){
            auto e = edges[i];
            if (info.features[i]->is_outlier_){
                e->computeError();
            }
            if (e->chi2() > chi2_th){
                info.features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outliers++;
            }else{
                info.features[i]->is_outlier_ = false;
                e->setLevel(0);
            }
            // remove the robust kernel to see if it's outlier
            if (iteration == num_iterations - 2){
                e->setRobustKernel(nullptr);
            }
        }
    }

    std::erase_if(info.features, [](const auto feature){
        return feature->is_outlier_;
    });

    return vertex_pose->estimate();
}