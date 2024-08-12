#include "map.h"
#include <cstdio>
#include <opencv2/highgui.hpp>

void Map::InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame)
{
    backend_finished_.acquire();

    current_keyframe_ = key_frame;
    all_key_frames_.insert({key_frame->key_frame_id_, key_frame});

    for(auto &feature: key_frame->features_left_){
        feature->map_point_.lock()->observers_.push_back({key_frame,feature});
    }

    semaphore_.release();
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

void Map::RemoveOutliers()
{
    for(auto &KF : all_key_frames_){
        auto kf = KF.second;
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