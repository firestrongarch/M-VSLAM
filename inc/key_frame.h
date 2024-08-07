#pragma once
#include "frame_base.h"

class KeyFrame:public FrameBase
{
public:
    using Ptr = std::shared_ptr<KeyFrame>;
    KeyFrame() = default;
    KeyFrame(std::shared_ptr<FrameBase> frame);

    unsigned long key_frame_id_;

};
