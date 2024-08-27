#pragma once
#include "frame_base.h"
#include "feature.h"
#include "frame.h"
#include <mutex>

class KeyFrame:public FrameBase
{
public:
    using Ptr = std::shared_ptr<KeyFrame>;
    KeyFrame() = default;
    KeyFrame(std::shared_ptr<Frame> frame);

    unsigned long Id() override{
        return key_frame_id_;
    }

    unsigned long key_frame_id_;

    cv::Mat left_image_;
    std::vector<std::shared_ptr<Feature>> features_left_;

    // 与回环产生数据竞争
    std::mutex mutex_features_left_;
};
