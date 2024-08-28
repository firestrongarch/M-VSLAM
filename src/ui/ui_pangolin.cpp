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

void UiPangolin::SaveTrajectoryTUM()
{
    std::ofstream outfile;
    outfile.open("../result.txt", std::ios_base::out | std::ios_base::trunc);
    outfile << std::fixed;
    std::map<unsigned long, KeyFrame::Ptr> poses_map;

    for (auto &kf : map_->GetAllKeyFrames()){
        unsigned long keyframe_id = kf.first;
        auto keyframe = kf.second;
        poses_map.insert(make_pair(keyframe_id, keyframe));
    }

    for (auto &kf : poses_map){
        unsigned long keyframe_id = kf.first;
        KeyFrame::Ptr keyframe = kf.second;
        double timestamp = keyframe->timestamp_;
        Sophus::SE3d frame_pose = keyframe->Pose().inverse();
        Eigen::Vector3d pose_t = frame_pose.translation();
        Eigen::Matrix3d pose_R = frame_pose.rotationMatrix();
        Eigen::Quaterniond pose_q = Eigen::Quaterniond(pose_R);

        outfile << std::setprecision(6) << timestamp << " " << pose_t.transpose().x() << " "
                << pose_t.transpose().y() << " " << pose_t.transpose().z() << " "
                << pose_q.coeffs().transpose().x() << " " << pose_q.coeffs().transpose().y()
                << " " << pose_q.coeffs().transpose().z() << " "
                << pose_q.coeffs().transpose().w() << std::endl;
    }
    outfile.close();
}