#pragma once
#include "frame_base.h"
#include "feature.h"
#include <memory>
#include <opencv2/core/mat.hpp>

class Frame : public FrameBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<Frame>;

    Frame() = default;
    Frame(const cv::Mat &left_img, const cv::Mat &right_img, const double &timestamp);
    
    // the relative pose to the reference KF
    void SetRelativePose(const Sophus::SE3d &relative_pose);
    const Sophus::SE3d& RelativePose() const;

    cv::Mat left_image_, right_image_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;
private:
    Sophus::SE3d relative_pose_;
    
};