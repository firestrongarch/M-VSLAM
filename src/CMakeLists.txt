add_library(M_VSLAM SHARED)

target_link_libraries(M_VSLAM PUBLIC
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

get_target_property(HEADERS M_VSLAM INTERFACE_SOURCES)

install(FILES ${HEADERS}
DESTINATION include/M_VSLAM
COMPONENT headers
)
install(TARGETS M_VSLAM EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
    COMPONENT libraries
)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)