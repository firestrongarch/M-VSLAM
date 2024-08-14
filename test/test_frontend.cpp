#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

import frontend;
import map;
import ui_pangolin;

using namespace std;
void LoadKittiImagesTimestamps(const string &str_path_to_sequence,
                                      vector<string> &str_image_left_vec_path,
                                      vector<string> &str_image_right_vec_path,
                                      vector<double> &timestamps_vec);
const std::string kitti_path{"/data/Kitti/00"};
int main(int argc, char const *argv[])
{
    auto file = cv::FileStorage("../config/kitti_00.yaml",
                                      cv::FileStorage::READ);
    Frontend frontend;
    // Backend backend;
    Map::Ptr map = std::make_shared<Map>();
    UiPangolin::Ptr ui_pangolin = std::make_shared<UiPangolin>();
    ui_pangolin->SetMap(map);
    // backend.SetMap(map);
    auto ui_pangolin_thread = std::thread(&UiPangolin::Run,ui_pangolin);
    // auto backend_thread = std::thread(&Backend::Run,backend);

    frontend.SetUiPangolin(ui_pangolin);

    cv::Mat K = file["cam0"]["K"];
    cv::Mat T_01 = file["cam1"]["T_01"];
    // file["cam0"]["K"] >> K;
    // file["cam1"]["T_01"] >> T_01;
    Eigen::Vector3d t_right = Eigen::Vector3d(T_01.at<double>(0,3), 0, 0);
    Camera::Ptr left = std::make_shared<Camera>(
        K,
        cv::Mat(),
        Sophus::SE3d()
    );
    Camera::Ptr right = std::make_shared<Camera>(
        K,
        cv::Mat(),
        Sophus::SE3d(Sophus::SO3d(), t_right)
    );

    map->left_camera_ = left;
    map->right_camera_ = right;
    frontend.SetMap(map);

    /// load sequence frames
    std::vector<std::string> image_left_vec_path, image_right_vec_path;
    std::vector<double> vec_timestamp;
    LoadKittiImagesTimestamps(kitti_path,image_left_vec_path,image_right_vec_path,vec_timestamp);

    const int num_images = image_left_vec_path.size();
    for (int ni = 0; ni < num_images; ni++)
    {
        // cout  << "Has processed " << ni + 1 << " frames." << std::endl;
        Core::Mat img_left = cv::imread(image_left_vec_path[ni]);
        Core::Mat img_right = cv::imread(image_right_vec_path[ni]);
        double timestamp = vec_timestamp[ni];

        frontend.RunBinocular(img_left, img_right,timestamp);
    }

    // backend_thread.join();
    ui_pangolin_thread.join();
    return 0;
}

void LoadKittiImagesTimestamps(const string &str_path_to_sequence,
                                      vector<string> &str_image_left_vec_path,
                                      vector<string> &str_image_right_vec_path,
                                      vector<double> &timestamps_vec)
{
    namespace fs = std::filesystem; 
    fs::path path_time_file(str_path_to_sequence + "/times.txt");

    std::optional<std::ifstream> fTimes = std::ifstream(path_time_file, ios::in | ios::app);

    std::optional<std::string> line{""};
    while (getline(fTimes.value(), line.value())){
        if (line){
            std::istringstream line_stream(line.value());
            double t;
            line_stream >> t;
            timestamps_vec.push_back(t);
        }
    }

    string strPrefixLeft = str_path_to_sequence + "/image_0/";
    string strPrefixRight = str_path_to_sequence + "/image_1/";

    for (int i = 0; i < timestamps_vec.size(); i++){
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        str_image_left_vec_path.push_back(strPrefixLeft + ss.str() + ".png");
        str_image_right_vec_path.push_back(strPrefixRight + ss.str() + ".png");
    }
}