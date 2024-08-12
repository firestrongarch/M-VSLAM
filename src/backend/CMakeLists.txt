target_sources(M_VSLAM 
PRIVATE
    backend.cpp
PUBLIC
    backend.h
)

target_include_directories(M_VSLAM PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)