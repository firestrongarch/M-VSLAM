module;
#include <memory>
#include <vector>
export module key_frame;
export import map_point;
export import frame_base;
export import frame;
export class KeyFrame:public FrameBase
{
public:
    using Ptr = std::shared_ptr<KeyFrame>;
    KeyFrame() = default;
    KeyFrame(std::shared_ptr<Frame> frame);

    unsigned long Id() override{
        return key_frame_id_;
    }

    unsigned long key_frame_id_;

    Core::Mat left_image_;
    std::vector<std::shared_ptr<Feature>> features_left_;
};