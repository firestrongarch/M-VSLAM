target_sources(M_VSLAM 
PRIVATE
    ui_pangolin_point_cloud.cpp
    ui_pangolin.cpp
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/ui_pangolin.h>
)

target_include_directories(M_VSLAM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)