#pragma once
#include <pangolin/pangolin.h>
#include "sophus/se3.hpp"
#include "map_base.h"

class UiPangolin {
public:
    using Ptr = std::shared_ptr<UiPangolin>;
    UiPangolin() = default;
    void Run();
    void AddTrajectoryPose(const Sophus::SE3d &pose);
    // void AddMapPoint(const Eigen::Vector3d &point);
    void RenderMapPoint();
    void SetMap(const MapBase::Ptr map);

private:
    void Render();
    std::vector<Eigen::Vector3f> traj_VO_;
    std::vector<Sophus::SE3f> poses_;                  /// pose
    std::vector<Eigen::Vector3f> point_cloud_;
    MapBase::Ptr map_;
};