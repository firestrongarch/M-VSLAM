#include "frontend.h"
// #include "algorithm.h"
#include <opencv2/calib3d.hpp>
import triangulate;

std::vector<Eigen::Vector3d> Frontend::Pixel2Camera(cv::Point2f const &pt1, cv::Point2f const &pt2)
{
    return std::vector<Eigen::Vector3d>{
        map_->left_camera_->Pixel2Camera(Eigen::Vector2d(pt1.x, pt1.y)),
        map_->right_camera_->Pixel2Camera(Eigen::Vector2d(pt2.x, pt2.y))
    };
}

int Frontend::Triangulation(TriInfo info)
{
    // step1: 获取相机位姿
    std::vector<Sophus::SE3d> poses{info.prev_pose, info.next_pose};

    // step2: 构建地图点 
    int i{-1};
    std::erase_if(info.prev_features,[&](auto feature){     
        i++;
        if(!feature->map_point_.expired()){
            return false;
        }

        auto points = Pixel2Camera(
            info.prev_features.at(i)->pt, 
            info.next_features.at(i)->pt
        );
        Eigen::Vector3d point3d = Eigen::Vector3d::Zero();

        if(triangulation(poses, points, point3d) && point3d[2] > 0){
            MapPoint::Ptr map_point(std::make_shared<MapPoint>(info.current_pose_Twc * point3d));
            feature->map_point_ = map_point;
            if(map_){
                map_->InsertMapPoint(map_point);
            }
        }else{
            feature->is_outlier_ = true;
        }
        // 未能三角化，删除该元素
        return feature->is_outlier_;
    });

    return info.next_features.size();
}