#pragma once
#include "key_frame.h"
#include "map_point.h"

class MapBase {
public:
    using Ptr = std::shared_ptr<MapBase>;
    using KeyFramesType = std::unordered_map<unsigned long, std::shared_ptr<KeyFrame>>;
    using MapPointType = std::unordered_map<unsigned long, std::shared_ptr<MapPoint>>;
    MapBase() = default;
    virtual void InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame){}
    virtual void InsertMapPoint(std::shared_ptr<MapPoint> map_point){}
    virtual const MapPointType& GetAllMapPoints() = 0;
};