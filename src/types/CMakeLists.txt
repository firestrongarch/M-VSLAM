target_sources(M_VSLAM 
PRIVATE
    feature.cpp
    map_point.cpp
    frame_base.cpp
    frame.cpp
    key_frame.cpp
    camera.cpp
    map.cpp
PUBLIC
    feature.h
    map_point.h
    frame_base.h
    frame.h
    key_frame.h
    camera.h
    map.h
    g2o_types.hpp
    algorithm.h
)

target_include_directories(M_VSLAM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
