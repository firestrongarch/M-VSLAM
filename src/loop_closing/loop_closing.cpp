#include "loop_closing.h"
#include "feature.h"
#include "lcdetector.h"
#include <cstdio>
#include <memory>
#include <opencv2/core/types.hpp>
#include <vector>
#include <opencv2/core/eigen.hpp>
#include "frontend.h"
#include "g2o_types.hpp"
void LoopClosing::Run()
{
    // Creating the loop closure detector object
    ibow_lcd::LCDetectorParams params;  // Assign desired parameters
    ibow_lcd::LCDetector lcdet(params);
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(300);
    while(map_->loop_closing_thread_)
    {
        map_->loop_closing_start_.acquire();
        auto kf = map_->current_keyframe_;
        std::vector<cv::KeyPoint> kps;
        for(auto& feature : kf->features_left_){
            kps.push_back(*feature);
        }
        ibow_lcd::LCDetectorResult result;
        cv::Mat descs;
        detector->compute(kf->left_image_, kps, descs);
        lcdet.process(kf->Id(), kps, descs, &result);
        switch (result.status) {
        case ibow_lcd::LC_DETECTED:{
            map_->loop_frame_id_ = kf->Id();
            map_->similar_frame_id_ = result.train_id;
            auto KFs = map_->GetAllKeyFrames();
            std::vector<Feature::Ptr> features;
            auto prev_features = KFs.at(result.train_id)->features_left_;
            auto inliners = Frontend::OpticalFlow({
                .prev_features = prev_features, 
                .next_features = features, 
                .prev_img = KFs.at(result.train_id)->left_image_, 
                .next_img = KFs.at(kf->Id())->left_image_
            });

            if(inliners > 300){
                std::cout <<" loop "<< inliners << " inliers" << std::endl;
                auto pose = Frontend::Optimize({
                    .features = features,
                    .pose = ComputeCorrectPose(features),
                    .K = map_->left_camera_->GetK()
                });
            
                for(auto feature : kf->features_left_){
                    auto mp = feature->map_point_.lock();
                    Eigen::Vector3d posCamera = kf->Pose() * mp->Pos();
                    mp->SetPosition(pose.inverse() * posCamera);
                }

                kf->SetPose(pose);
                kf->relative_pose_to_loop_KF_ = kf->Pose().inverse() * KFs.at(result.train_id)->Pose().inverse() ;
                map_->loop_frame_id_ = kf->Id();
                map_->similar_frame_id_ = result.train_id;

                PoseGraphOptimization();

                map_->loop_corrected_= true;
                map_->loop_closing_thread_ = false;
            }
            break;
        }
        default:
            // std::cout << "No status information" << std::endl;
            break;
        }

        map_->loop_closing_finished_.release();
    }
}

Sophus::SE3d LoopClosing::ComputeCorrectPose(std::vector<std::shared_ptr<Feature>> &features)
{
    std::vector<cv::Point3f> loop_point3f;
    std::vector<cv::Point2f> current_point2f;

    for(auto feature : features){
        if(feature->map_point_.lock()){
            Eigen::Vector3d p = *feature->map_point_.lock();
            loop_point3f.push_back(cv::Point3f(p.x(), p.y(), p.z()));
            current_point2f.push_back(feature->pt);
        }
    }
    cv::Mat rvec, tvec, R, K;
    cv::eigen2cv(map_->left_camera_->GetK(), K);
    cv::solvePnPRansac(
        loop_point3f, 
        current_point2f, 
        K, cv::Mat(), rvec, tvec, false, 100, 5.991, 0.99
    );

    Eigen::Matrix3d Reigen;
    Eigen::Vector3d teigen;

    cv::Rodrigues(rvec, R);
    cv::cv2eigen(R, Reigen);
    cv::cv2eigen(tvec, teigen);

    auto pose = Sophus::SE3d(Reigen, teigen);

    return pose;
}

void LoopClosing::PoseGraphOptimization()
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
    std::shared_ptr<KeyFrame> kf = map_->current_keyframe_;
    while(true){
        auto id = kf->Id();
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(kf->key_frame_id_);
        vertex_pose->setEstimate(kf->Pose());
        vertex_pose->setMarginalized(false);
        if ( id == map_->loop_frame_id_ || id == map_->similar_frame_id_ || id==0){
            vertex_pose->setFixed(true);
        }
        optimizer.addVertex(vertex_pose);
        vertices_kf.insert({kf->key_frame_id_, vertex_pose});

        if(kf->last_key_frame_.expired()){
            break;
        }
        kf = kf->last_key_frame_.lock();
    }

    // edges
    int index = 0;
    kf = map_->current_keyframe_;
    while(true){
        if(kf->last_key_frame_.expired()){
            break;
        }

        if(kf == map_->current_keyframe_){
            auto similar_frame = allKFs.at(map_->similar_frame_id_);
            EdgePoseGraph *edge = new EdgePoseGraph();
            edge->setId(index);
            edge->setVertex(0, vertices_kf.at(kf->Id()));
            edge->setVertex(1, vertices_kf.at(similar_frame->Id()));
            edge->setMeasurement(kf->relative_pose_to_last_KF_);
            edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
            optimizer.addEdge(edge);
            index++;
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
    }

    std::puts("pose graph optimization start");
    // do the optimization
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    std::puts("pose graph optimization finished");

    // set the KFs' optimized poses
    for (auto &v : vertices_kf){
        allKFs.at(v.first)->SetPose(v.second->estimate());
    }

} // mutex