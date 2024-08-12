add_library(M_VSLAM SHARED "")

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
add_subdirectory(frontend)
add_subdirectory(backend)
add_subdirectory(ui)
