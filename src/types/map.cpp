#include "map.h"
#include <opencv2/highgui.hpp>

void Map::InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame)
{
    current_keyframe_ = key_frame;
    all_key_frames_.insert({key_frame->key_frame_id_, key_frame});
}

void Map::InsertMapPoint(std::shared_ptr<MapPoint> map_point)
{
    if (all_map_points_.find(map_point->id_) == all_map_points_.end()){
        all_map_points_.insert(make_pair(map_point->id_, map_point));
    }
}

const Map::MapPoints& Map::GetAllMapPoints()
{
    return all_map_points_;
}

void Map::ShowCurrentKeyFrame()
{
    cv::imshow("current keyframe", current_keyframe_->left_image_);
}