target_sources(M_VSLAM 
PUBLIC FILE_SET CXX_MODULES FILES
    triangulate.ixx
    frame_base.ixx
    map_point.ixx
    feature.ixx
    frame.ixx
    key_frame.ixx
    camera.ixx
    map.ixx
    g2o_types.ixx
)

target_include_directories(M_VSLAM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
