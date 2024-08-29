#include "key_frame.h"
#include "frame_base.h"

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame):FrameBase(*frame)
{
    static unsigned long Id = 0;
    key_frame_id_ = Id++;

    left_image_ = frame->left_image_;
    features_left_ = frame->features_left_;
}


std::vector<std::shared_ptr<Feature>> KeyFrame::GetFeatures(){
    std::lock_guard<std::mutex> lock(mutex_features_left_);
    return features_left_;
}

void KeyFrame::SetFeatures(const std::vector<std::shared_ptr<Feature>>& features){
    std::lock_guard<std::mutex> lock(mutex_features_left_);
    features_left_ = features;
}