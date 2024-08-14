module;
#include <sophus/se3.hpp>
export module frame_base;

export class FrameBase{
public:
    FrameBase() = default;
    void SetPose(const Sophus::SE3d &pose){
        pose_ = pose;
    }
    Sophus::SE3d Pose(){
        return pose_;
    }

    virtual unsigned long Id(){
        return frame_id_;
    };

    unsigned long frame_id_{};
    double timestamp_{};
    
protected:
    Sophus::SE3d pose_;  // T_cw
};