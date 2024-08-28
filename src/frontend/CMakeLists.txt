target_sources(M_VSLAM 
PRIVATE
    frontend.cpp
    frontend_init.cpp
    frontend_track.cpp
    frontend_show.cpp
    frontend_utils.cpp
    frontend_template.hpp
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/frontend.h>
)  

target_include_directories(M_VSLAM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)