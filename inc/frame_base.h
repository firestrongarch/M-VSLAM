#pragma once
#include "feature.h"
#include "sophus/se3.hpp"
#include <opencv2/core/mat.hpp>

class FrameBase{
public:
    FrameBase() = default;
    void SetPose(const Sophus::SE3d &pose);
    Sophus::SE3d Pose();

    unsigned long frame_id_{};
    double timestamp_{};

    cv::Mat left_image_;
    std::vector<std::shared_ptr<Feature>> features_left_;
protected:
    Sophus::SE3d pose_;  // T_cw
};

