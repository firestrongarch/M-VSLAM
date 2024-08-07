add_library(slam_types SHARED
            frame_base.cpp
            frame.cpp
            key_frame.cpp
            feature.cpp
            map_point.cpp
            map.cpp)
target_link_libraries(slam_types opencv_core)

# set(CPPCHECK_EXECUTABLE "/usr/bin/cppcheck")
# # 将cppcheck添加到编译目标之后
# add_custom_command(TARGET slam_data
#                    POST_BUILD
#                    COMMAND ${CPPCHECK_EXECUTABLE} --enable=all --suppress=missingIncludeSystem
#                            --std=c++23 --force --quiet --project=/home/fu/cpp/project3/build/compile_commands.json
#                    COMMENT "Running cppcheck after build"
# )