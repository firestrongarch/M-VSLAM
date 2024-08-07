add_library(slam_frontend SHARED
            frontend.cpp
            frontend_utils.cpp
            frontend_init.cpp
            frontend_show.cpp
            frontend_track.cpp
            frontend_optimize.cpp
            camera.cpp)
target_link_libraries(slam_frontend slam_types 
    opencv_highgui opencv_features2d opencv_video pango_display fmt
    g2o_core glog
    )

target_precompile_headers(slam_frontend PRIVATE ${PROJECT_SOURCE_DIR}/inc/frontend.h ${PROJECT_SOURCE_DIR}/inc/algorithm.h)
# set(CPPCHECK_EXECUTABLE "/usr/bin/cppcheck")
# # 将cppcheck添加到编译目标之后
# add_custom_command(TARGET slam_data
#                    POST_BUILD
#                    COMMAND ${CPPCHECK_EXECUTABLE} --enable=all --suppress=missingIncludeSystem
#                            --std=c++23 --force --quiet --project=/home/fu/cpp/project3/build/compile_commands.json
#                    COMMENT "Running cppcheck after build"
# )