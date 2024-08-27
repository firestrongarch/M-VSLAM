#include "loop_closing.h"
#include "lcdetector.h"
#include <memory>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <vector>

void LoopClosing::Run()
{
    // Creating the loop closure detector object
    ibow_lcd::LCDetectorParams params;  // Assign desired parameters
    ibow_lcd::LCDetector lcdet(params);
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(300);
    while(true)
    {
        map_->loop_closing_start_.acquire();
        auto keyframe = map_->current_keyframe_;

        std::unique_lock lock{keyframe->mutex_features_left_};
        std::vector<cv::KeyPoint> kps;
        for(auto& feature : keyframe->features_left_){
            cv::KeyPoint kp;
            kp.pt = feature->pt;
            kps.push_back(kp);
        }
        lock.unlock();
        
        ibow_lcd::LCDetectorResult result;
        cv::Mat descs;
        detector->compute(keyframe->left_image_, kps, descs);
        lcdet.process(keyframe->Id(), kps, descs, &result);
        switch (result.status) {
        case ibow_lcd::LC_DETECTED:
            std::cout << "--- Loop detected!!!: " << result.train_id <<
                        " with " << result.inliers << " inliers" << std::endl;
            break;
        case ibow_lcd::LC_NOT_DETECTED:
            std::cout << "No loop found" << std::endl;
            break;
        case ibow_lcd::LC_NOT_ENOUGH_IMAGES:
            std::cout << "Not enough images to found a loop" << std::endl;
            break;
        case ibow_lcd::LC_NOT_ENOUGH_ISLANDS:
            std::cout << "Not enough islands to found a loop" << std::endl;
            break;
        case ibow_lcd::LC_NOT_ENOUGH_INLIERS:
            std::cout << "Not enough inliers" << std::endl;
            break;
        case ibow_lcd::LC_TRANSITION:
            std::cout << "Transitional loop closure" << std::endl;
            break;
        default:
            std::cout << "No status information" << std::endl;
            break;
        }

        map_->loop_closing_finished_.release();
    }
}