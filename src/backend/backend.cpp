#include "backend.h"
#include "g2o_types.hpp"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <iostream>

void Backend::Run()
{
    while (true)
    {
        map_->semaphore_.acquire();
        std::cout << "ID: "<< map_->current_keyframe_->frame_id_ << std::endl;
    }
}

void Backend::OptimizeActiveMap()
{
    g2o::SparseOptimizer optimizer;
    using BlockSolver = g2o::BlockSolver_6_3;
    using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;
    auto solver = g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>());
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
    optimizer.setAlgorithm(algorithm);

    auto KFs = map_->GetAllKeyFrames();
    auto MPs = map_->GetAllMapPoints();

    std::unordered_map<unsigned long, VertexPose *> vertices_kfs;
    unsigned long max_kf_id = 0;
    for(auto &KF : KFs){
        auto kf = KF.second;
        VertexPose *vertex = new VertexPose();
        vertex->setId(kf->key_frame_id_);
        vertex->setEstimate(kf->Pose());
        optimizer.addVertex(vertex);
        vertices_kfs.insert({kf->key_frame_id_, vertex});

        max_kf_id = std::max(max_kf_id, kf->key_frame_id_);
    }

    Eigen::Matrix3d cam_K = map_->left_camera_->GetK();

    int index = 1;
    double chi2_th = 5.891;

    std::unordered_map<unsigned long, VertexXYZ *> vertices_mps;
    std::unordered_map<EdgePoseXYZ *, Feature::Ptr> edges_and_features;
    for(auto &MP: MPs){
        auto mp = MP.second;
        VertexXYZ *v = new VertexXYZ;
        v->setEstimate(mp->Pos());
        v->setId((max_kf_id + 1) + mp->id_); /// avoid vertex id equal
        v->setMarginalized(true);
        vertices_mps.insert({mp->id_, v});
        optimizer.addVertex(v);

        // // edges
        // for(auto &obs: mp->observations_){
        // }
    }
}

void Backend::SetMap(const Map::Ptr map)
{
    map_ = map;
}