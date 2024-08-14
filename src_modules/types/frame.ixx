module;
#include <sophus/se3.hpp>
#include <memory>
export module frame;
export import core;
export import frame_base;
export import map_point;

export class Frame : public FrameBase
{
public:
    using Ptr = std::shared_ptr<Frame>;

    Frame() = default;
    Frame(const Core::Mat &left_img, const Core::Mat &right_img, const double &timestamp);
    
    // the relative pose to the reference KF
    void SetRelativePose(const Sophus::SE3d &relative_pose);
    const Sophus::SE3d& RelativePose() const;

    Core::Mat left_image_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    Core::Mat right_image_;
    std::vector<std::shared_ptr<Feature>> features_right_;
private:
    Sophus::SE3d relative_pose_;
    
};