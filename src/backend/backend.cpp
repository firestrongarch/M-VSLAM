#include "backend.h"
#include "g2o_types.hpp"
#include "key_frame.h"
#include "map_point.h"
#include <cstddef>
#include <cstdio>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <iostream>
#include <memory>

void Backend::Run()
{
    while (true)
    {
        map_->backend_start_.acquire();
        if(map_->loop_closing_thread_ ){
            map_->loop_closing_finished_.acquire();
        }
        OptimizeMap();

        // if(map_->loop_corrected_){
        //     PoseGraphOptimization();
        //     map_->loop_corrected_=false;
        // }

        map_->backend_finished_.release();
    }
}

void Backend::OptimizeMap()
{
    g2o::SparseOptimizer optimizer;
    using BlockSolver = g2o::BlockSolver_6_3;
    using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;
    auto solver = g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>());
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
    optimizer.setAlgorithm(algorithm);

    auto KFs = map_->GetActiveKeyFrames();
    auto MPs = map_->GetActiveMapPoints();

    std::unordered_map<unsigned long, VertexPose *> vertices_kfs;
    unsigned long max_kf_id = 0;
    for(auto &KF : KFs){
        auto kf = KF.second;
        VertexPose *vertex = new VertexPose();
        vertex->setId(kf->key_frame_id_);
        vertex->setEstimate(kf->Pose());
        if(kf->Id() == map_->loop_frame_id_ && map_->loop_frame_id_ != 0 && map_->loop_corrected_){
            vertex->setFixed(true);
            map_->loop_corrected_ = false;
        }
        optimizer.addVertex(vertex);

        max_kf_id = std::max(max_kf_id, kf->key_frame_id_);
        vertices_kfs.insert({kf->key_frame_id_, vertex});
    }

    Eigen::Matrix3d cam_K = map_->left_camera_->GetK();
    Sophus::SE3d pose = map_->left_camera_->GetPose();

    int index = 1;
    double chi2_th = 5.891;

    std::unordered_map<unsigned long, VertexXYZ *> vertices_mps;
    std::unordered_map<EdgePoseXYZ *, std::weak_ptr<MapPoint>> edges_and_mps;
    for(auto &MP: MPs){
        auto mp = MP.second;
        VertexXYZ *v = new VertexXYZ;
        v->setEstimate(mp->Pos());
        v->setId((max_kf_id + 1) + mp->id_); /// avoid vertex id equal
        v->setMarginalized(true);

        // 局部地图优化需要判断地图点是否在活跃关键帧内
        // v->setFixed(true);

        vertices_mps.insert({mp->id_, v});
        optimizer.addVertex(v);

        // edges
        for(auto &ob: mp->observers_){ 
            auto kf = ob.frame.lock();
            if(KFs.find(kf->Id()) == KFs.end()){
                continue;
            }
            // if(kf->Id() == map_->loop_frame_id_ && map_->loop_frame_id_ != 0){
            //     v->setFixed(true);
            // }
            auto& pt = ob.kp.lock()->pt;
            auto edge = new EdgePoseXYZ(cam_K, pose);
            edge->setId(index);
            edge->setVertex(0, vertices_kfs.at(kf->Id()));
            edge->setVertex(1, v);
            edge->setMeasurement(Eigen::Vector2d(pt.x, pt.y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_mps.insert({edge, mp});
            optimizer.addEdge(edge);
            index++;
        }
    }

    // optimize
    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;

    while (iteration < 5){
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &em : edges_and_mps){
            if (em.first->chi2() > chi2_th){
                cnt_outlier++;
            }else{
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.7){
            break;
        }else{
            // chi2_th *= 2;
            iteration++;
        }
    }

    // process the outlier edges
    for (auto &em : edges_and_mps){
        if (em.first->chi2() > chi2_th){
            em.second.lock()->is_outlier_ = true;
        }
    }

    for (auto &v : vertices_kfs){
        KFs.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_mps){
        MPs.at(v.first)->SetPosition(v.second->estimate());
    }

    map_->RemoveOutliers();
}

void Backend::SetMap(const Map::Ptr map)
{
    map_ = map;
    map_->backend_thread_ = true;
}

void Backend::PoseGraphOptimization()
{
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    Map::KeyFrames allKFs = map_->GetAllKeyFrames();

    // vertices
    std::map<unsigned long, VertexPose *> vertices_kf;
    while(true){
        static std::shared_ptr<KeyFrame> kf;
        if(kf == nullptr){
            kf = map_->current_keyframe_;
        }
        auto id = kf->Id();
        std::cout<< "id" << id << std::endl;

        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(kf->key_frame_id_);
        vertex_pose->setEstimate(kf->Pose());
        vertex_pose->setMarginalized(false);
        if ( id == map_->loop_frame_id_ || id == map_->similar_frame_id_){
            vertex_pose->setFixed(true);
        }
        optimizer.addVertex(vertex_pose);
        vertices_kf.insert({kf->key_frame_id_, vertex_pose});

        if(kf->last_key_frame_.expired()){
            kf = nullptr;
            break;
        } else {
            kf = kf->last_key_frame_.lock();
        }
    }

    // edges
    int index = 0;
    while(true){
        static std::shared_ptr<KeyFrame> kf;
        if(kf == nullptr){
            kf = map_->current_keyframe_;
        }
        EdgePoseGraph *edge = new EdgePoseGraph();
        edge->setId(index);
        edge->setVertex(0, vertices_kf.at(kf->Id()));
        edge->setVertex(1, vertices_kf.at(kf->last_key_frame_.lock()->Id()));
        edge->setMeasurement(kf->relative_pose_to_last_KF_);
        edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
        optimizer.addEdge(edge);
        index++;
        kf = kf->last_key_frame_.lock();
        if(kf->last_key_frame_.expired()){
            kf = nullptr;
            break;
        } else {
            kf = kf->last_key_frame_.lock();
        }
    }
    // do the optimization
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    std::puts("pose graph optimization finished");

    // set the KFs' optimized poses
    for (auto &v : vertices_kf){
        allKFs.at(v.first)->SetPose(v.second->estimate());
    }

} // mutex
