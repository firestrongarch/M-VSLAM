#include "key_frame.h"
#include "frame_base.h"

KeyFrame::KeyFrame(std::shared_ptr<FrameBase> frame)
{
    static unsigned long Id = 0;
    key_frame_id_ = Id++;

}
