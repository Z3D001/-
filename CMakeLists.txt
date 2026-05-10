cmake_minimum_required(VERSION 3.12)
project(sigle_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(Threads REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

link_directories(${ZMQ_LIBRARY_DIRS})

# RadarConnect 库
add_library(radarconnect STATIC src/radarconnect.cpp)
target_include_directories(radarconnect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(radarconnect rclcpp)

#发送节点 
add_executable(sigle_radar_splicing_node src/sigle_radar_splicing_node.cpp)
ament_target_dependencies(sigle_radar_splicing_node
  rclcpp nav_msgs std_msgs pcl_conversions
)
target_link_libraries(sigle_radar_splicing_node
  radarconnect
  ${PCL_LIBRARIES}
  ${ZMQ_LIBRARIES}
  Threads::Threads
)

# 接收节点 
add_executable(pointcloud_receiver_node src/pointcloud_receiver_node.cpp)
ament_target_dependencies(pointcloud_receiver_node rclcpp)

target_link_libraries(pointcloud_receiver_node
  ${ZMQ_LIBRARIES}
  Threads::Threads
)

install(TARGETS
  radarconnect
  sigle_radar_splicing_node
  pointcloud_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()