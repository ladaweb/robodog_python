cmake_minimum_required(VERSION 3.16)
project(slam_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(
    /usr/local/include
    /usr/local/include/ddscxx
    /usr/local/include/iceoryx/v2.0.2
    include
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    /usr/local/lib
    /usr/local/lib/dds_backup
    /unitree/module/unitree_slam/lib
)

add_executable(qr_test src/qr_test.cpp)
target_link_libraries(qr_test
    ${OpenCV_LIBS}
)

add_executable(keyDemo src/keyDemo.cpp)
target_link_libraries(keyDemo
    ${OpenCV_LIBS}
    /usr/local/lib/libunitree_go2_sdk.a
    /usr/local/lib/libunitree_go2_idl_cpp.a
    /usr/local/lib/libunitree_ros2_idl_cpp.a
    /usr/local/lib/dds_backup/libddsc.so
    /usr/local/lib/dds_backup/libddscxx.so
    pthread
)

set_target_properties(keyDemo PROPERTIES
    BUILD_RPATH "/usr/local/lib/dds_backup"
    INSTALL_RPATH "/usr/local/lib/dds_backup"
)