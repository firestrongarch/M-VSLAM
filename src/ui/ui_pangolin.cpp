#include "ui_pangolin.h"

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

        Render();
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

void UiPangolin::AddTrajectoryPose(const Sophus::SE3d &pose)
{
    poses_.emplace_back(pose.cast<float>());
    traj_VO_.emplace_back(pose.translation().cast<float>());
}

void UiPangolin::SetMap(const MapBase::Ptr map)
{
    map_ = map;
}