module;
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
export module ui_pangolin;
import map;

export class UiPangolin {
public:
    using Ptr = std::shared_ptr<UiPangolin>;
    UiPangolin() = default;
    void Run();
    void AddTrajectoryPose(const Sophus::SE3d &pose);
    // void AddMapPoint(const Eigen::Vector3d &point);
    void RenderMapPoint();
    void SetMap(const Map::Ptr map);

private:
    void Render();
    void RenderKf();
    std::vector<Eigen::Vector3f> traj_VO_;
    std::vector<Sophus::SE3f> poses_;                  /// pose
    std::vector<Eigen::Vector3f> point_cloud_;
    Map::Ptr map_;
};

void UiPangolin::Run()
{
    pangolin::CreateWindowAndBind("my_slam",1280,720);
    glEnable(GL_DEPTH_TEST);

    /// Issue specific OpenGl we might need
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1280,720,5000,5000,640,360,1.0,1e10),
        pangolin::ModelViewLookAt(0,1000,0, 0,0,0, pangolin::AxisZ)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() ){
        // Clear entire screen 
        glClearColor(1, 1, 1, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

        // Render();
        RenderKf();
        RenderMapPoint();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

void UiPangolin::Render()
{
    if (traj_VO_.empty()){
        return;
    }
    glPointSize(5);
    glBegin(GL_POINTS);
    for (auto &p : traj_VO_){
        glColor3f(0, 1, 0);
        glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
}

void UiPangolin::RenderKf()
{
    glPointSize(5);
    glBegin(GL_POINTS);
    auto const Kfs = map_->GetAllKeyFrames();
    for (auto &kf : Kfs){
        auto p = kf.second->Pose().inverse().translation();
        glColor3f(0, 1, 0);
        glVertex3d(p.x(), p.y(), p.z());
    }
    glEnd();
}

void UiPangolin::AddTrajectoryPose(const Sophus::SE3d &pose)
{
    poses_.emplace_back(pose.cast<float>());
    traj_VO_.emplace_back(pose.translation().cast<float>());
}

void UiPangolin::SetMap(const Map::Ptr map)
{
    map_ = map;
}