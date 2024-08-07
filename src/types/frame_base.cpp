#include "frame_base.h"

void FrameBase::SetPose(const Sophus::SE3d &pose)
{
    std::unique_lock<std::mutex> lck(update_pose_);
    pose_ = pose;
}

Sophus::SE3d FrameBase::Pose()
{
    std::unique_lock<std::mutex> lck(update_pose_);
    return pose_;
}