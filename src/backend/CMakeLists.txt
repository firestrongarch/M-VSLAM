add_library(slam_backend SHARED
            backend.cpp
            )
target_link_libraries(slam_frontend slam_types 
    opencv_highgui opencv_features2d opencv_video pango_display fmt
    g2o_core glog g2o_solver_csparse
    )