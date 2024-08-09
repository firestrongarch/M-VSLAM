#pragma once

#include <memory>
#include <semaphore>
#include "map_base.h"
#include "camera.h"

class Map : public MapBase
{
public:
    using Ptr = std::shared_ptr<Map>;

    Map() = default;

    void InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame);
    void InsertMapPoint(std::shared_ptr<MapPoint> map_point);
    const MapPoints& GetAllMapPoints();

    void ShowCurrentKeyFrame();


    std::binary_semaphore semaphore_{0};
    std::shared_ptr<KeyFrame> current_keyframe_{nullptr};

    std::shared_ptr<Camera> left_camera_;
    std::shared_ptr<Camera> right_camera_;

private:
    MapPoints all_map_points_;
    MapPoints active_map_point_;

    KeyFrames all_key_frames_;
    KeyFrames active_key_frames_;

    unsigned int num_active_key_frames_;
};

