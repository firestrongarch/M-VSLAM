
add_executable(test_frontend test_frontend.cpp)
target_link_libraries(test_frontend slam_frontend slam_ui slam_backend)

add_executable(yaml yaml.cpp)
target_link_libraries(yaml ${OpenCV_LIBS})