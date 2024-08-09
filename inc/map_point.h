#pragma once

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include "frame_base.h"
#include <list>

class MapPoint:public Eigen::Vector3d
{
public:
    using Ptr = std::shared_ptr<MapPoint>;
    MapPoint() = default;
    MapPoint(Eigen::Vector3d position);

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

    std::list<std::weak_ptr<FrameBase>> observers_;
private:
    std::mutex mutex_pos_;
};

