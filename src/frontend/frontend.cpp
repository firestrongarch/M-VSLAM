#include "frontend.h"
#include "frontend_template.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

void Frontend::RunBinocular(const cv::Mat &left_image, const cv::Mat &right_iamge, const double timestamp)
{
    current_frame_ = std::make_shared<Frame>(left_image, right_iamge, timestamp);

    switch (track_status_)
    {
    case INIT:
        Init();
        break;
    case GOOD:
        Track();
        break;
    default:
        break;
    }

    ui_pangolin_->AddTrajectoryPose(current_frame_->Pose().inverse());
    last_frame_ = current_frame_;
}


void Frontend::Track()
{
    // T_cw = T_cc * T_cw
    current_frame_->SetPose(relative_motion_ * last_frame_->Pose() );
    // step1 跟踪上一帧
    OpticalFlow({
        .prev_features = last_frame_->features_left_, 
        .next_features = current_frame_->features_left_, 
        .prev_img = last_frame_->left_image_, 
        .next_img = current_frame_->left_image_
    });

    // 显示跟踪结果
    cv::Mat show = current_frame_->left_image_.clone();
    for (size_t i = 0; i < current_frame_->features_left_.size(); i++){
        const cv::Point2i pt1 = current_frame_->features_left_.at(i)->pt;
        const cv::Point2i pt2 = last_frame_->features_left_.at(i)->pt;
        cv::circle(show, pt1, 2, cv::Scalar(0, 250, 0), 2);
        cv::line(show, pt1, pt2, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("LK", show);
    map_->ShowCurrentKeyFrame();
    cv::waitKey(10);

    // 最小化重投影误差
    Optimize({
        .features = current_frame_->features_left_,
        .pose = current_frame_->Pose(),
        .K = map_->left_camera_->GetK()
    });

    // T_cc = T_cw * T_wc
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    // step2 在当前帧中补充更多特征点
    if(current_frame_->features_left_.size() < 100){
        DetectFeatures({
            .img = current_frame_->left_image_, 
            .features = current_frame_->features_left_
        });
        OpticalFlow({
                .prev_features = current_frame_->features_left_ , 
                .next_features = current_frame_->features_right_, 
                .prev_img = current_frame_->left_image_, 
                .next_img = current_frame_->right_image_
        });
        Triangulation({
            .prev_features = current_frame_->features_left_, 
            .next_features = current_frame_->features_right_,
            .prev_pose = map_->left_camera_->GetPose(),
            .next_pose = map_->right_camera_->GetPose(),
            .current_pose_Twc = current_frame_->Pose().inverse()
        });

        map_->InsertKeyFrame(std::make_shared<KeyFrame>(current_frame_));
    }
}