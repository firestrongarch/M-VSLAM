#pragma once

#include <memory>
#include <mutex>
#include <semaphore>
#include "camera.h"
#include "key_frame.h"
#include "map_point.h"

class Map
{
public:
    using Ptr = std::shared_ptr<Map>;
    using KeyFrames = std::unordered_map<unsigned long, std::shared_ptr<KeyFrame>>;
    using MapPoints = std::unordered_map<unsigned long, std::shared_ptr<MapPoint>>;
    Map() = default;

    void InsertKeyFrame(std::shared_ptr<KeyFrame> key_frame);
    void InsertMapPoint(std::shared_ptr<MapPoint> map_point);
    MapPoints GetAllMapPoints();
    KeyFrames GetAllKeyFrames();

    MapPoints GetActiveMapPoints();
    KeyFrames GetActiveKeyFrames();

    void ShowCurrentKeyFrame();

    void RemoveOutliers();

    struct OptimizeInfo{
        std::vector<std::shared_ptr<Feature>>& features;
        Sophus::SE3d const & pose;
    };
    Sophus::SE3d Optimize(OptimizeInfo info);

    bool backend_thread_{false};
    std::binary_semaphore backend_start_{0};
    std::binary_semaphore backend_finished_{1};

    bool loop_closing_thread_{false};
    std::binary_semaphore loop_closing_start_{0};
    std::binary_semaphore loop_closing_finished_{1};

    std::shared_ptr<KeyFrame> current_keyframe_{nullptr};

    std::shared_ptr<Camera> left_camera_;
    std::shared_ptr<Camera> right_camera_;

private:
    MapPoints all_map_points_;
    KeyFrames all_key_frames_;

    KeyFrames active_key_frames_;
    // std::deque<std::shared_ptr<KeyFrame>> active_key_frames_;

    std::mutex mutex_map_points_;
    unsigned int num_active_key_frames_;
};

