cmake_minimum_required(VERSION 3.5)
project(lidar_test)

set(CMAKE_CXX_STANDARD 14)

# Pakai include path yang benar dari versi SDK terbaru
include_directories(
  /usr/local/include/src
  /usr/local/include/core
)

# Library SDK terbaru sudah terpasang di /usr/local/lib
link_directories(/usr/local/lib)

add_executable(test test.cpp)
target_link_libraries(test ydlidar_sdk pthread)

