#include "frame.h"

static unsigned long FrmaeId = 0;
Frame::Frame(const cv::Mat &left_image, const cv::Mat &right_image, const double &timestamp)
{
    left_image_ = left_image;
    right_image_ = right_image;
    timestamp_ = timestamp;

    frame_id_ = FrmaeId++;
}

void Frame::SetRelativePose(const Sophus::SE3d &relative_pose)
{
    relative_pose_ = relative_pose;
}

const Sophus::SE3d& Frame::RelativePose() const
{
    return relative_pose_;
}