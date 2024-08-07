#include "frame_base.h"

void FrameBase::SetPose(const Sophus::SE3d &pose)
{
    pose_ = pose;
}

Sophus::SE3d FrameBase::Pose()
{
    return pose_;
}