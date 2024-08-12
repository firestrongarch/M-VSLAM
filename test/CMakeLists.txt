add_executable(test_frontend test_frontend.cpp)
target_link_libraries(test_frontend M_VSLAM)

add_executable(yaml yaml.cpp)
target_link_libraries(yaml ${OpenCV_LIBS})