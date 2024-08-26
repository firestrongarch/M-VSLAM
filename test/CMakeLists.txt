add_executable(test_frontend test_frontend.cpp)
target_link_libraries(test_frontend M_VSLAM)

add_executable(test_lc test_lc.cpp)
target_link_libraries(test_lc M_VSLAM)