cmake_minimum_required(VERSION 3.5.0)
project(hikvision_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDES})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDES})

catkin_package(INCLUDE_DIRS include)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(hik_ros src/hik_ros.cpp)

target_link_libraries(hik_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  hcnetsdk PlayCtrl
)
