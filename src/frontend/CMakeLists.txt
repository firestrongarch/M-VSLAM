add_library(slam_frontend SHARED
            frontend.cpp
            frontend_utils.cpp
            frontend_init.cpp
            frontend_show.cpp
            frontend_track.cpp
            frontend_optimize.cpp
            camera.cpp)
target_link_libraries(slam_frontend slam_types 
    opencv_highgui opencv_features2d opencv_video pango_display fmt
    g2o_core glog
    )
