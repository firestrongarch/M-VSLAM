target_sources(M_VSLAM 
PUBLIC FILE_SET CXX_MODULES FILES
    frontend.ixx
    detect.ixx
)

target_include_directories(M_VSLAM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)