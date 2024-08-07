#include "camera.h"

Eigen::Vector3d Camera::World2Camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w)
{
    return pose_ * T_c_w * p_w;
}

Eigen::Vector2d Camera::World2Pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w)
{
    return Camera2Pixel(World2Camera(p_w, T_c_w));
}

Eigen::Vector3d Camera::Camera2World(const Eigen::Vector3d &p_c, const Sophus::SE3d &T_c_w)
{
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Eigen::Vector2d Camera::Camera2Pixel(const Eigen::Vector3d &p_c)
{
    auto z = p_c(2,0);
    Eigen::Vector3d pt = K_ * p_c /z;
    return Eigen::Vector2d(pt(0, 0),
                            pt(1, 0));
}

Eigen::Vector3d Camera::Pixel2Camera(const Eigen::Vector2d &p_p, double depth)
{
    Eigen::Vector3d p_c = depth * K_.inverse() * Eigen::Vector3d(p_p(0, 0), p_p(1, 0), 1); 
    return p_c;
}

Eigen::Vector3d Camera::Pixel2World(const Eigen::Vector2d &p_p, const Sophus::SE3d &T_c_w,
                                    double depth)
{
    return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
}
