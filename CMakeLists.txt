cmake_minimum_required(VERSION 3.5.0)
project(hikvision_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  camera_info_manager
)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDES})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDES})

catkin_package(INCLUDE_DIRS include)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_library(hik_cam src/hik_cam.cpp)
target_link_libraries(hik_cam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    hcnetsdk PlayCtrl
)

add_executable(hik_cam_node node/hik_cam_node.cpp)
target_link_libraries(hik_cam_node hik_cam)