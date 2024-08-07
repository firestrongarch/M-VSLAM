add_library(slam_ui SHARED
            ui_pangolin.cpp
            ui_pangolin_point_cloud.cpp)
target_link_libraries(slam_ui pango_display fmt)

# set(CPPCHECK_EXECUTABLE "/usr/bin/cppcheck")
# # 将cppcheck添加到编译目标之后
# add_custom_command(TARGET slam_data
#                    POST_BUILD
#                    COMMAND ${CPPCHECK_EXECUTABLE} --enable=all --suppress=missingIncludeSystem
#                            --std=c++23 --force --quiet --project=/home/fu/cpp/project3/build/compile_commands.json
#                    COMMENT "Running cppcheck after build"
# )