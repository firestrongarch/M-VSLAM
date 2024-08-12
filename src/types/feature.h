#pragma once
#include "map_point.h"
#include <memory>
#include <opencv2/core/types.hpp>

class Feature : public cv::KeyPoint
{
public:
    using Ptr = std::shared_ptr<Feature>;
    Feature() = default;
    Feature(const cv::KeyPoint &kp);

    std::weak_ptr<MapPoint> map_point_;

    bool is_outlier_ = false;
};

