#pragma once
#include "sophus/se3.hpp"
#include <opencv2/core/mat.hpp>

class FrameBase{
public:
    FrameBase() = default;
    void SetPose(const Sophus::SE3d &pose);
    Sophus::SE3d Pose();

    virtual unsigned long Id(){
        return frame_id_;
    };

    unsigned long frame_id_{};
    double timestamp_{};
    
protected:
    Sophus::SE3d pose_;  // T_cw
};

