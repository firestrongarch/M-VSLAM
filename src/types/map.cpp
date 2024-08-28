#include "map.h"
#include <cstdio>
#include <mutex>
#include <opencv2/highgui.hpp>

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

void Map::InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame)
{
    if(backend_thread_){
        backend_finished_.acquire();
    }
    if(loop_closing_thread_){
        loop_closing_finished_.acquire();
    }
    
    current_keyframe_ = key_frame;
    all_key_frames_.insert({key_frame->key_frame_id_, key_frame});
    active_key_frames_.insert({key_frame->key_frame_id_, key_frame});
    if (active_key_frames_.size() >= 10) {
        active_key_frames_.erase(current_keyframe_->Id() - 10);
    }

    for(auto &feature: key_frame->features_left_){
        feature->map_point_.lock()->observers_.push_back({key_frame,feature});
    }

    backend_start_.release();
    loop_closing_start_.release();
}

void Map::InsertMapPoint(std::shared_ptr<MapPoint> map_point)
{
    if (all_map_points_.find(map_point->id_) == all_map_points_.end()){
        all_map_points_.insert(make_pair(map_point->id_, map_point));
    }
}

Map::MapPoints Map::GetAllMapPoints()
{
    return all_map_points_;
}

Map::KeyFrames Map::GetAllKeyFrames()
{
    return all_key_frames_;
}

Map::KeyFrames Map::GetActiveKeyFrames()
{
    return active_key_frames_;
}

Map::MapPoints Map::GetActiveMapPoints()
{
    MapPoints active_map_points;
    for(auto &kf : active_key_frames_){
        for(auto &feature : kf.second->features_left_){
            active_map_points.insert({feature->map_point_.lock()->id_, feature->map_point_.lock()});
        }
    }
    return active_map_points;
}

void Map::RemoveOutliers()
{
    for(auto &KF : all_key_frames_){
        auto kf = KF.second;
        std::lock_guard lock{kf->mutex_features_left_};
        std::erase_if(kf->features_left_, [](auto &f){
            return f->map_point_.lock()->is_outlier_;
        });
    }

    std::erase_if(all_map_points_, [](auto &mp){
        if(mp.second->is_outlier_){
            std::puts("erase outliers");
        }
        return mp.second->is_outlier_;
    });
}

void Map::ShowCurrentKeyFrame()
{
    cv::imshow("current keyframe", current_keyframe_->left_image_);
}

Sophus::SE3d Map::Optimize(OptimizeInfo info)
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
    for(auto &feature:info.features){
        std::shared_ptr<MapPoint> map_point = feature->map_point_.lock();
        if(map_point->is_outlier_){
            std::cout<<"map point is outlier"<<std::endl;   
            continue;
        }
        auto pt = feature->pt;
        auto edge = new EdgePose(map_point->Pos(),left_camera_->GetK());
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

    std::erase_if(info.features, [](const auto &feature){
        return feature->is_outlier_;
    });

    return vertex_pose->estimate();
}