module;
#include <memory>
#include <vector>
#include <algorithm>
export module detect;
import map_point;

template<class T = Feature>
struct DetectorInfo{
    Core::Mat& img;
    std::vector<std::shared_ptr<T>>& features;
    inline static Core::Feature2D detector = Core::ORB::create(300);
};

template<class T = Feature>
int DetectFeatures(DetectorInfo<T> info)
{
    // step1 屏蔽已有特征点的区域
    Core::Mat mask(info.img.size(), Core::_8UC1, Core::ScalarAll(255));
    for (const auto &feat : info.features){
        Core::rectangle(mask,
                    feat->pt - Core::Point2f(10, 10),
                    feat->pt + Core::Point2f(10, 10),
                    0,
                    Core::FILLED);
    }

    // step2 在已存在特征点的区域外进行特征点检测
    std::vector<Core::KeyPoint> kps;
    info.detector->detect(info.img, kps,mask);

    // step3 对检测到的特征点创建Feature对象并加入当前帧
    std::ranges::for_each(kps,[&](Core::KeyPoint kp){
        std::shared_ptr<T> feature(std::make_shared<T>(kp));
        info.features.emplace_back(feature);
    });

    return 0;
}