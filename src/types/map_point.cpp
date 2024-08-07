#include "map_point.h"

MapPoint::MapPoint(Eigen::Vector3d position): Eigen::Vector3d(position)
{
    static unsigned long Id = 0;
    id_ = Id++;
}
