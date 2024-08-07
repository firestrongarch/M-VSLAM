#pragma once

#include <Eigen/Core>
#include <memory>
#include <mutex>

class MapPoint:public Eigen::Vector3d
{
public:
    using Ptr = std::shared_ptr<MapPoint>;
    MapPoint() = default;
    MapPoint(Eigen::Vector3d position);

    Eigen::Vector3d Pos(){
        std::unique_lock<std::mutex> lck(update_get_mutex_);
        return *this;
    }

    void SetPosition(const Eigen::Vector3d &position){
        std::unique_lock<std::mutex> lck(update_get_mutex_);
        Eigen::Vector3d* base = this; 
        *base = position;
    }

    unsigned long id_ = 0;
    int active_observed_times_ = 0;
    int observed_times_ = 0;
    bool is_outlier_ = false;
private:
    std::mutex update_get_mutex_;
    // std::list<std::weak_ptr<Feature>> active_observations_;
    // std::list<std::weak_ptr<Feature>> observations_;
};

