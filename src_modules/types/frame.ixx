module;
#include <memory>
#include <opencv2/core/types.hpp>
#include <sophus/se3.hpp>
export module frame;
import feature;
import frame_base;

export class Frame : public FrameBase
{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<Frame>;

    Frame() = default;
    Frame(const cv::Mat &left_img, const cv::Mat &right_img, const double &timestamp){
        static unsigned long FrmaeId = 0;
        left_image_ = left_img;
        right_image_ = right_img;
        timestamp_ = timestamp;

        frame_id_ = FrmaeId++;
    }
    
    // the relative pose to the reference KF
    void SetRelativePose(const Sophus::SE3d &relative_pose){
        relative_pose_ = relative_pose;
    }
    const Sophus::SE3d& RelativePose() const{
        return relative_pose_;
    }

    cv::Mat left_image_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    cv::Mat right_image_;
    std::vector<std::shared_ptr<Feature>> features_right_;
private:
    Sophus::SE3d relative_pose_;
    
};