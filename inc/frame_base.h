#pragma once
#include "sophus/se3.hpp"
#include <mutex>

class FrameBase{
public:
    FrameBase() = default;
    void SetPose(const Sophus::SE3d &pose);
    Sophus::SE3d Pose();

    unsigned long frame_id_{};
    double timestamp_{};
protected:
    Sophus::SE3d pose_;  // T_cw
    std::mutex update_pose_;
};

