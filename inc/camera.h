#pragma once
#include "sophus/se3.hpp"
#include "opencv2/core/mat.hpp"
#include <opencv2/core/eigen.hpp>
#include <memory>

class Camera {
public:
    using Ptr = std::shared_ptr<Camera>;
    Camera() = default;
    Camera(double fx, double fy, double cx, double cy, double baseline, const Sophus::SE3d &pose,
         const cv::Mat dist_coef)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose), dist_coef_(dist_coef)
    {
        K_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    }

    Camera(const cv::Mat& K, const cv::Mat& dist_coef , const Sophus::SE3d &pose):pose_(pose){
        cv::cv2eigen(K, K_);
    }

    const Sophus::SE3d& GetPose() const { return pose_; }
    double GetBaseline() const { return baseline_; }
    Eigen::Matrix3d GetK() const { return K_; }

    Eigen::Vector3d World2Camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);
    Eigen::Vector2d World2Pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w);

    Eigen::Vector3d Camera2World(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w);
    Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d &p_c);
    
    Eigen::Vector3d Pixel2Camera(const Eigen::Vector2d &p_p, double depth = 1);
    Eigen::Vector3d Pixel2World(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w,
                                double depth = 1);

private:
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    double baseline_ = 0;
    Sophus::SE3d pose_;
    Sophus::SE3d pose_inv_;
    cv::Mat dist_coef_;
    Eigen::Matrix3d K_;
};
