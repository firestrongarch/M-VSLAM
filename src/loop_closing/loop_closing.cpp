#include "loop_closing.h"
#include "feature.h"
#include "lcdetector.h"
#include <cstdio>
#include <memory>
#include <opencv2/core/types.hpp>
#include <vector>
#include "frontend.h"
#include <opencv2/core/eigen.hpp>
void LoopClosing::Run()
{
    // Creating the loop closure detector object
    ibow_lcd::LCDetectorParams params;  // Assign desired parameters
    ibow_lcd::LCDetector lcdet(params);
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(300);
    while(true)
    {
        map_->loop_closing_start_.acquire();
        auto keyframe = map_->current_keyframe_;

        std::vector<cv::KeyPoint> kps;
        for(auto& feature : keyframe->features_left_){
            kps.push_back(*feature);
        }
        
        ibow_lcd::LCDetectorResult result;
        cv::Mat descs;
        detector->compute(keyframe->left_image_, kps, descs);
        lcdet.process(keyframe->Id(), kps, descs, &result);
        switch (result.status) {
        case ibow_lcd::LC_DETECTED:{
            auto KFs = map_->GetAllKeyFrames();
            std::vector<Feature::Ptr> features;
            auto prev_features = KFs.at(keyframe->Id())->features_left_;
            auto inliners = Frontend::OpticalFlow({
                .prev_features = prev_features, 
                .next_features = features, 
                .prev_img = KFs.at(keyframe->Id())->left_image_, 
                .next_img = KFs.at(result.train_id)->left_image_
            });

            if(inliners > 350){
                std::cout <<" loop "<< inliners << " inliers" << std::endl;
                auto pose = Frontend::Optimize({
                    .features = features,
                    .pose = ComputeCorrectPose(features),
                    .K = map_->left_camera_->GetK()
                });
                keyframe->SetPose(pose);
                map_->loop_frame_id_ = keyframe->Id();
                map_->loop_corrected_= true;
                std::puts("corrected!");
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