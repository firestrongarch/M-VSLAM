module;
#include <sophus/se3.hpp>
#include <memory>
#include <print>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
export module frontend;
import map;
import frame;
import key_frame;
import g2o_types;
import triangulate;
import ui_pangolin;
import detect;

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

bool Frontend::Init()
{
    // step1: detect features
    int cnt_detected_features = DetectFeatures({
        .img = current_frame_->left_image_, .features = current_frame_->features_left_,
    });
    int cnt_track_features = OpticalFlow({
        .prev_features = current_frame_->features_left_ , .next_features = current_frame_->features_right_, 
        .prev_img = current_frame_->left_image_, .next_img = current_frame_->right_image_});
    if (cnt_track_features < 100){
        std::println("Too few feature points");
        return false;
    }

    // step2: create map
    if (InitMap()){
        track_status_ = GOOD;
        map_->InsertKeyFrame(std::make_shared<KeyFrame>(current_frame_));
        return true;
    }

    return false;
}

bool Frontend::InitMap()
{
    // 三角化左右图像的特征点
    int nums = Triangulation({
        .prev_features = current_frame_->features_left_,
        .next_features = current_frame_->features_right_,
        .prev_pose = map_->left_camera_->GetPose(),
        .next_pose = map_->right_camera_->GetPose()
    });

    // 检测是否有足够多的点
    if (nums < 50){
        std::println("Too few feature points: {}",nums);
        return false;
    }else{
        std::println("Init map success: {}",nums);
    }

    return true;
}

void Frontend::SetMap(const Map::Ptr map)
{
    map_ = map;
}

void Frontend::SetUiPangolin(const UiPangolin::Ptr ui_pangolin)
{
    ui_pangolin_ = ui_pangolin;
}

int Frontend::Optimize(OptimizeInfo info)
{
    g2o::SparseOptimizer optimizer;
    using BlockSolver = g2o::BlockSolver_6_3;
    using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
    auto solver = g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>());
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
    optimizer.setAlgorithm(algorithm);

    auto vertex_pose = new VertexPose;
    vertex_pose->setId(0);

    vertex_pose->setEstimate(info.pose);
    optimizer.addVertex(vertex_pose);

    // edges
    int index = 1;
    std::vector<EdgePose *> edges;
    for(auto &feature:info.features){
        std::shared_ptr<MapPoint> map_point = feature->map_point_.lock();
        if(map_point->is_outlier_){
            std::cout<<"map point is outlier"<<std::endl;   
            continue;
        }
        auto pt = feature->pt;
        auto edge = new EdgePose(map_point->Pos(),info.K);
        edge->setId(index);
        edge->setVertex(0,vertex_pose);
        edge->setMeasurement(Eigen::Vector2d(pt.x, pt.y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber);
        edges.emplace_back(edge);
        optimizer.addEdge(edge);
        index++;
    }

    // estimate the Pose and determine the outliers
    // start optimization
    const double chi2_th = 5.991;
    int cnt_outliers = 0;
    int num_iterations = 4;
    for (int iteration = 0; iteration < num_iterations; iteration++){
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outliers = 0;

        for (size_t i = 0, N = edges.size(); i < N; i++){
            auto e = edges[i];
            if (info.features[i]->is_outlier_){
                e->computeError();
            }
            if (e->chi2() > chi2_th){
                info.features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outliers++;
            }else{
                info.features[i]->is_outlier_ = false;
                e->setLevel(0);
            }
            // remove the robust kernel to see if it's outlier
            if (iteration == num_iterations - 2){
                e->setRobustKernel(nullptr);
            }
        }
    }

    std::erase_if(info.features, [](const auto &feature){
        return feature->is_outlier_;
    });

    /// set pose
    current_frame_->SetPose(vertex_pose->estimate());

    return info.features.size();
}

int Frontend::OpticalFlow(LkInfo info)
{
    std::vector<Core::Point2f> prev_points, next_points;
    for(const auto feature : info.prev_features){
        prev_points.push_back(feature->pt);
    }
    std::vector<unsigned char> status;
    Core::Mat error;
    Core::calcOpticalFlowPyrLK(
        info.prev_img,
        info.next_img,
        prev_points,
        next_points,
        status,
        error
    );

    size_t i = 0;
    std::erase_if(info.prev_features,[&](auto feature){
        if (status.at(i)){
            Feature::Ptr next_feature = std::make_shared<Feature>();
            next_feature->pt = next_points.at(i);
            next_feature->map_point_ = feature->map_point_;

            info.next_features.emplace_back(next_feature);
        }else{
            feature->is_outlier_ = true;
        }
        i++;
        return feature->is_outlier_;
    });

    return info.next_features.size();
}

std::vector<Eigen::Vector3d> Frontend::Pixel2Camera(Core::Point2f const &pt1, Core::Point2f const &pt2)
{
    return std::vector<Eigen::Vector3d>{
        map_->left_camera_->Pixel2Camera(Eigen::Vector2d(pt1.x, pt1.y)),
        map_->right_camera_->Pixel2Camera(Eigen::Vector2d(pt2.x, pt2.y))
    };
}

int Frontend::Triangulation(TriInfo info)
{
    // step1: 获取相机位姿
    std::vector<Sophus::SE3d> poses{info.prev_pose, info.next_pose};

    // step2: 构建地图点 
    int i{-1};
    std::erase_if(info.prev_features,[&](auto feature){     
        i++;
        if(!feature->map_point_.expired()){
            return false;
        }

        auto points = Pixel2Camera(
            info.prev_features.at(i)->pt, 
            info.next_features.at(i)->pt
        );
        Eigen::Vector3d point3d = Eigen::Vector3d::Zero();

        if(triangulation(poses, points, point3d) && point3d[2] > 0){
            MapPoint::Ptr map_point(std::make_shared<MapPoint>(info.current_pose_Twc * point3d));
            feature->map_point_ = map_point;
            if(map_){
                map_->InsertMapPoint(map_point);
            }
        }else{
            feature->is_outlier_ = true;
        }
        // 未能三角化，删除该元素
        return feature->is_outlier_;
    });

    return info.next_features.size();
}

void Frontend::RunBinocular(const Core::Mat &left_image, const Core::Mat &right_iamge, const double timestamp)
{
    current_frame_ = std::make_shared<Frame>(left_image, right_iamge, timestamp);

    switch (track_status_)
    {
    case INIT:
        Init();
        break;
    case GOOD:
        Track();
        break;
    default:
        break;
    }

    ui_pangolin_->AddTrajectoryPose(current_frame_->Pose().inverse());
    last_frame_ = current_frame_;
}


void Frontend::Track()
{
    // T_cw = T_cc * T_cw
    current_frame_->SetPose(relative_motion_ * last_frame_->Pose() );
    // step1 跟踪上一帧
    OpticalFlow({
        .prev_features = last_frame_->features_left_, 
        .next_features = current_frame_->features_left_, 
        .prev_img = last_frame_->left_image_, 
        .next_img = current_frame_->left_image_
    });

    // 显示跟踪结果
    // Core::Mat show = current_frame_->left_image_.clone();
    // for (size_t i = 0; i < current_frame_->features_left_.size(); i++){
    //     const Core::Point2i pt1 = current_frame_->features_left_.at(i)->pt;
    //     const Core::Point2i pt2 = last_frame_->features_left_.at(i)->pt;
    //     Core::circle(show, pt1, 2, cv::Scalar(0, 250, 0), 2);
    //     Core::line(show, pt1, pt2, cv::Scalar(0, 0, 255), 1);
    // }
    // cv::imshow("LK", show);
    // map_->ShowCurrentKeyFrame();
    // cv::waitKey(10);

    // 最小化重投影误差
    Optimize({
        .features = current_frame_->features_left_,
        .pose = current_frame_->Pose(),
        .K = map_->left_camera_->GetK()
    });

    // T_cc = T_cw * T_wc
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    // step2 在当前帧中补充更多特征点
    if(current_frame_->features_left_.size() < 100){
        DetectFeatures({
            .img = current_frame_->left_image_, 
            .features = current_frame_->features_left_
        });
        OpticalFlow({
                .prev_features = current_frame_->features_left_ , 
                .next_features = current_frame_->features_right_, 
                .prev_img = current_frame_->left_image_, 
                .next_img = current_frame_->right_image_
        });
        Triangulation({
            .prev_features = current_frame_->features_left_, 
            .next_features = current_frame_->features_right_,
            .prev_pose = map_->left_camera_->GetPose(),
            .next_pose = map_->right_camera_->GetPose(),
            .current_pose_Twc = current_frame_->Pose().inverse()
        });

        map_->InsertKeyFrame(std::make_shared<KeyFrame>(current_frame_));
    }
}