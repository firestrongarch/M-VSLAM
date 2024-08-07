#include<opencv2/opencv.hpp>
#include<iostream>

int main()
{
    cv::FileStorage fs("../config/kitti_00 copy.yaml", cv::FileStorage::READ);
    cv::Mat matrix_eye;
    fs["cam0"]["K"] >> matrix_eye;
    std::cout<<matrix_eye<<std::endl;
}