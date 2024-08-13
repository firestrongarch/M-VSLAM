module;
#include "sophus/se3.hpp"
#include <opencv2/core/mat.hpp>

export module frame_base;

export class FrameBase{
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

void FrameBase::SetPose(const Sophus::SE3d &pose)
{
    pose_ = pose;
}

Sophus::SE3d FrameBase::Pose()
{
    return pose_;
}