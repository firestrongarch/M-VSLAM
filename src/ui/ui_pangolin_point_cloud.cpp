#include "ui_pangolin.h"

// void UiPangolin::AddMapPoint(const Eigen::Vector3d &point)
// {
//     point_cloud_.push_back(point);
// }

void UiPangolin::RenderMapPoint()
{
    float blue[3] = {0, 0, 1};
    float red[3] = {1, 0, 0};
    if (map_ == nullptr)
        return;

    glPointSize(2);
    glBegin(GL_POINTS);

    for (auto &mp : map_->GetAllMapPoints())
    {
        auto pos = mp.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }

    glEnd();
}