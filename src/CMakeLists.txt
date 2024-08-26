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
add_subdirectory(loop_closing)
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

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "lib/cmake/M_VSLAM"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${Tutorial_VERSION_MAJOR}.${Tutorial_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/M_VSLAMConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/M_VSLAMConfigVersion.cmake
  DESTINATION lib/cmake/M_VSLAM
)

export(EXPORT ${PROJECT_NAME}Targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)