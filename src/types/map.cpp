#include "map.h"
#include <cstdio>
#include <opencv2/highgui.hpp>

void Map::InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame)
{
    // if(backend_thread_){
    //     backend_finished_.acquire();
    // }
    if(loop_closing_thread_){
        loop_closing_finished_.acquire();
    }

    if(current_keyframe_){
        key_frame->last_key_frame_ = current_keyframe_;
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
        // std::lock_guard lock{kf->mutex_features_left_};
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