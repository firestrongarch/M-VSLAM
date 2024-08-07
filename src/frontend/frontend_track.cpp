#include "feature.h"
#include "frontend.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

int Frontend::OpticalFlow(LkInfo info)
{
    std::vector<cv::Point2f> prev_points, next_points;
    for(const auto feature : info.prev_features){
        prev_points.push_back(feature->pt);
    }
    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        info.prev_img,
        info.next_img,
        prev_points,
        next_points,
        status,
        error
    );

    size_t i = 0;
    std::erase_if(info.prev_features,[&](auto feature){
        if (status.at(i)){
            Feature::Ptr next_feature = std::make_shared<Feature>();
            next_feature->pt = next_points.at(i);
            next_feature->map_point_ = feature->map_point_;
            info.next_features.emplace_back(next_feature);
        }else{
            feature->is_outlier_ = true;
        }
        i++;
        return feature->is_outlier_;
    });

    return info.next_features.size();
}
