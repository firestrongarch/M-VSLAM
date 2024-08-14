target_sources(M_VSLAM 
PUBLIC FILE_SET CXX_MODULES FILES
    core.ixx
    camera.ixx
    triangulate.ixx
    g2o_types.ixx
    frame_base.ixx
    map_point.ixx
    frame.ixx
    key_frame.ixx
    map.ixx
)

target_include_directories(M_VSLAM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
