add_library(M_VSLAM SHARED)
target_sources(M_VSLAM PUBLIC FILE_SET CXX_MODULES)

target_link_libraries(M_VSLAM
    ${OpenCV_LIBS}
    pango_display 
    fmt
    glog 
    g2o_core
    g2o_stuff
    g2o_solver_csparse
    g2o_csparse_extension
)

add_subdirectory(types)
add_subdirectory(ui)
add_subdirectory(frontend)
