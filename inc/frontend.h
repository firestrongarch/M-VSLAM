#pragma once

#include "frame.h"
#include "camera.h"
#include "map.h"
#include "ui_pangolin.h"
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

enum TrackStatus{INIT,GOOD,BAD,LOST};

class Frontend
{
public:
    Frontend(/* args */) = default;
    void SetCamera(const Camera::Ptr &left, const Camera::Ptr &right);
    void RunBinocular(const cv::Mat &left_image, const cv::Mat &right_iamge,
                      const double timestamp);
    void SetMap(const Map::Ptr map);
    void SetUiPangolin(const UiPangolin::Ptr ui_pangolin);

private:                
    bool Init();
    bool InitMap();
    
    void Track();

    struct LkInfo{
        std::vector<std::shared_ptr<Feature>>& prev_features;
        std::vector<std::shared_ptr<Feature>>& next_features;
        cv::Mat& prev_img;
        cv::Mat& next_img;
    };

    int OpticalFlow(LkInfo info);

    struct TriInfo{
        std::vector<std::shared_ptr<Feature>>& prev_features;
        std::vector<std::shared_ptr<Feature>>& next_features;
        Sophus::SE3d const & prev_pose;
        Sophus::SE3d const & next_pose;
        Sophus::SE3d current_pose_Twc{};
    };
    int Triangulation(TriInfo info);

    struct OptimizeInfo{
        std::vector<std::shared_ptr<Feature>>& features;
        Sophus::SE3d const & pose;
    };
    int Optimize(OptimizeInfo info);

    std::vector<Eigen::Vector3d> Pixel2Camera(cv::Point2f const &pt1, cv::Point2f const &pt2);

    void Show();

private:
    TrackStatus track_status_{INIT};
    std::shared_ptr<Camera> left_camera_;
    std::shared_ptr<Camera> right_camera_;
    std::shared_ptr<Frame> last_frame_;
    std::shared_ptr<Frame> current_frame_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<UiPangolin> ui_pangolin_;

    Sophus::SE3d relative_motion_;
};
