module;
#include <sophus/se3.hpp>
#include <memory>
export module frontend;
import map;
import frame;
import key_frame;
import g2o_types;
import triangulate;
import ui_pangolin;

enum TrackStatus{INIT,GOOD,BAD,LOST};

export class Frontend
{
public:
    Frontend(/* args */) = default;
    void RunBinocular(const Core::Mat &left_image, const Core::Mat &right_iamge,
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
        Core::Mat& prev_img;
        Core::Mat& next_img;
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
        const Eigen::Matrix3d& K;
    };
    int Optimize(OptimizeInfo info);

    std::vector<Eigen::Vector3d> Pixel2Camera(Core::Point2f const &pt1, Core::Point2f const &pt2);

    void Show();

private:
    TrackStatus track_status_{INIT};
    std::shared_ptr<Frame> last_frame_;
    std::shared_ptr<Frame> current_frame_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<UiPangolin> ui_pangolin_;

    Sophus::SE3d relative_motion_;
};
