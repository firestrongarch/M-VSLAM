module;
#include <opencv2/core/saturate.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
export module detect;
import feature;

template<class T = Feature>
struct DetectorInfo{
    cv::Mat& img;
    std::vector<std::shared_ptr<T>>& features;
    inline static cv::Ptr<cv::Feature2D> detector = cv::ORB::create(300);
};

template<class T = Feature>
int DetectFeatures(DetectorInfo<T> info)
{
    // step1 屏蔽已有特征点的区域
    cv::Mat mask(info.img.size(), CV_8UC1, cv::Scalar::all(255));
    for (const auto &feat : info.features){
        cv::rectangle(mask,
                    feat->pt - cv::Point2f(10, 10),
                    feat->pt + cv::Point2f(10, 10),
                    0,
                    cv::FILLED);
    }

    // step2 在已存在特征点的区域外进行特征点检测
    std::vector<cv::KeyPoint> kps;
    info.detector->detect(info.img, kps,mask);

    // step3 对检测到的特征点创建Feature对象并加入当前帧
    std::ranges::for_each(kps,[&](cv::KeyPoint kp){
        cv::Ptr<T> feature(std::make_shared<T>(kp));
        info.features.emplace_back(feature);
    });

    return 0;
}