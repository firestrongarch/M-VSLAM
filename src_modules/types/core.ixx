module;
#include <functional>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
export module core;

export namespace Core{
using Feature2D = cv::Ptr<cv::Feature2D>;
using ORB = cv::ORB;
using KeyPoint = cv::KeyPoint;
using Point2f = cv::Point2f;

const auto _8UC1 = CV_8UC1;
auto ScalarAll = cv::Scalar::all;

auto Wait = cv::waitKey;
auto cv2eigen = [](const cv::Mat& src, Eigen::Matrix3d& dst){
    cv::cv2eigen(src, dst);
};

const auto FILLED = cv::FILLED;

auto rectangle = [](cv::Mat& img, const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0){
    cv::rectangle(img, pt1, pt2, color, thickness, lineType, shift);
};

class Mat : public cv::Mat{
public:
    Mat() = default;
    Mat(const cv::String& path){
        cv::imread(path).copyTo(*this);
    }
    void show(){
        cv::imshow("Mat", *this);
    }
};
}
