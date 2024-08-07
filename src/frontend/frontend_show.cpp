#include "frontend.h"
#include <opencv2/opencv.hpp>

void Frontend::Show() 
{
    // show left and right image ORB
    cv::Mat left_img = current_frame_->left_image_.clone();
    cv::Mat right_img = current_frame_->right_image_.clone();
    auto f = [&](std::shared_ptr<cv::KeyPoint> feature, cv::Mat& img){
        if(feature == nullptr){
            return;
        }
        const float r = 5;
        const auto& pt = feature->pt;
        cv::Point2f pt1, pt2;
        pt1.x = pt.x - r;
        pt1.y = pt.y - r;
        pt2.x = pt.x + r;
        pt2.y = pt.y + r;

        cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0));
        cv::circle(img, pt, 2, cv::Scalar(0, 255, 0), -1);
    };

    auto& left_features = current_frame_->features_left_;
    auto& right_features = current_frame_->features_right_;
    
    std::ranges::for_each(left_features, [&](std::shared_ptr<cv::KeyPoint> feature){ f(feature, left_img); });
    std::ranges::for_each(right_features, [&](std::shared_ptr<cv::KeyPoint> feature){
        if (feature != nullptr) { // 只在特征点非空时绘制
            f(feature, right_img);
        }
    });

    std::vector<cv::Mat> images{left_img, right_img};
    cv::Mat show_img;
    cv::vconcat(images, show_img);

    cv::namedWindow("orb_detect_result", cv::WINDOW_NORMAL);
    cv::resizeWindow("orb_detect_result", cv::Size(800, 600));
    cv::imshow("orb_detect_result", show_img);

    if (true){
        cv::Mat show = current_frame_->left_image_.clone();
        // cv::cvtColor(current_frame_->left_image_, show, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < current_frame_->features_left_.size(); i++){
            // 判断非空
            if (current_frame_->features_right_.at(i)){
                const cv::Point2i pt1 = current_frame_->features_left_.at(i)->pt;
                const cv::Point2i pt2 = current_frame_->features_right_.at(i)->pt;
                cv::circle(show, pt1, 2, cv::Scalar(0, 250, 0), 2);
                cv::line(show, pt1, pt2, cv::Scalar(0, 0, 255), 1);
            }
        }
        cv::imshow("LK", show);
    }
    cv::waitKey(10);
}
