#include "frontend.h"
#include "frontend_template.hpp"
#include "key_frame.h"
#include "map.h"
#include <memory>
#include <print>

bool Frontend::Init()
{
    // step1: detect features
    int cnt_detected_features = DetectFeatures({
        .img = current_frame_->left_image_, .features = current_frame_->features_left_,
    });
    int cnt_track_features = OpticalFlow({
        .prev_features = current_frame_->features_left_ , .next_features = current_frame_->features_right_, 
        .prev_img = current_frame_->left_image_, .next_img = current_frame_->right_image_});
    if (cnt_track_features < 100){
        std::println("Too few feature points");
        return false;
    }

    // step2: create map
    if (InitMap()){
        track_status_ = GOOD;
        map_->InsertKeyFrame(std::make_shared<KeyFrame>(current_frame_));
        return true;
    }

    return false;
}

bool Frontend::InitMap()
{
    // 三角化左右图像的特征点
    int nums = Triangulation({
        .prev_features = current_frame_->features_left_,
        .next_features = current_frame_->features_right_,
        .prev_pose = map_->left_camera_->GetPose(),
        .next_pose = map_->right_camera_->GetPose()
    });

    // 检测是否有足够多的点
    if (nums < 50){
        std::println("Too few feature points: {}",nums);
        return false;
    }else{
        std::println("Init map success: {}",nums);
    }

    return true;
}

void Frontend::SetMap(const Map::Ptr map)
{
    map_ = map;
}

void Frontend::SetUiPangolin(const UiPangolin::Ptr ui_pangolin)
{
    ui_pangolin_ = ui_pangolin;
}
