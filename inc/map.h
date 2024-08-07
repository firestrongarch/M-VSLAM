#pragma once

#include <memory>
#include "map_base.h"

class Map : public MapBase
{
public:
    using Ptr = std::shared_ptr<Map>;

    Map() = default;

    void InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame);
    void InsertMapPoint(std::shared_ptr<MapPoint> map_point);
    const MapPointType& GetAllMapPoints();

private:
    MapPointType all_map_points_;
    MapPointType active_map_point_;

    KeyFramesType all_key_frames_;
    KeyFramesType active_key_frames_;

    std::shared_ptr<KeyFrame> current_keyframe_{nullptr};
    unsigned int num_active_key_frames_;
};

