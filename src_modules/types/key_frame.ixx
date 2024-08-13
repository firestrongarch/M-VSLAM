module;
#include <memory.h>
#include <vector>
#include <opencv2/core/types.hpp>
export module key_frame;
import feature;
import frame;
import frame_base;

export class KeyFrame:public FrameBase
{
public:
    using Ptr = std::shared_ptr<KeyFrame>;
    KeyFrame() = default;
    KeyFrame(std::shared_ptr<Frame> frame):FrameBase(*frame){
        static unsigned long Id = 0;
        key_frame_id_ = Id++;

        left_image_ = frame->left_image_;
        features_left_ = frame->features_left_;
    }   

    unsigned long Id() override{
        return key_frame_id_;
    }

    unsigned long key_frame_id_;

    cv::Mat left_image_;
    std::vector<std::shared_ptr<Feature>> features_left_;
};
