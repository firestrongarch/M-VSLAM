#include "key_frame.h"
#include "frame_base.h"

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame):FrameBase(*frame)
{
    static unsigned long Id = 0;
    key_frame_id_ = Id++;

    left_image_ = frame->left_image_;
    features_left_ = frame->features_left_;
}
