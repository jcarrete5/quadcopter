add_library(KalmanFilter INTERFACE) 
target_include_directories(KalmanFilter INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(KalmanFilter INTERFACE Eigen3::Eigen)

add_subdirectory(test)
