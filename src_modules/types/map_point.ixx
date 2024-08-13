module;
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include "opencv2/core/types.hpp"
#include <list>

export module map_point;

import frame_base;

export class MapPoint:public Eigen::Vector3d
{
public:
    using Ptr = std::shared_ptr<MapPoint>;
    MapPoint() = default;
    MapPoint(Eigen::Vector3d position): Eigen::Vector3d(position)
    {
        static unsigned long Id = 0;
        id_ = Id++;
    }

    Eigen::Vector3d Pos(){
        std::unique_lock<std::mutex> lck(mutex_pos_);
        return *this;
    }

    void SetPosition(const Eigen::Vector3d &position){
        std::unique_lock<std::mutex> lck(mutex_pos_);
        Eigen::Vector3d* base = this; 
        *base = position;
    }

    unsigned long id_ = 0;
    bool is_outlier_ = false;

    struct Observer{
        std::weak_ptr<FrameBase> frame;
        std::weak_ptr<cv::KeyPoint> kp;
    };
    std::list<Observer> observers_;
private:
    std::mutex mutex_pos_;
};