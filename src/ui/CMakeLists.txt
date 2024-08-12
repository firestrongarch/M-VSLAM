target_sources(M_VSLAM 
PRIVATE
    ui_pangolin_point_cloud.cpp
    ui_pangolin.cpp
PUBLIC
    ui_pangolin.h
)

target_include_directories(M_VSLAM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)