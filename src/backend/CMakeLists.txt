target_sources(M_VSLAM
PRIVATE
    backend.cpp
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/backend.h>
)

target_include_directories(M_VSLAM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
    $<INSTALL_INTERFACE:include/M_VSLAM>
)