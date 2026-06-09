cmake_minimum_required(VERSION 3.14)
project(arena_gz_plugins)

find_package(ament_cmake REQUIRED)

# gz-sim 8 (Harmonic)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# gz-math 7 (Pose3d / Quaterniond)
find_package(gz-math7 REQUIRED)

# ROS 2
find_package(rclcpp REQUIRED)
find_package(arena_people_msgs REQUIRED)

add_library(PedSkeletonPlugin SHARED
  src/PedSkeletonPlugin.cc
)
target_include_directories(PedSkeletonPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(PedSkeletonPlugin PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-math7::gz-math7
  rclcpp::rclcpp
  ${arena_people_msgs_TARGETS}
)

install(TARGETS PedSkeletonPlugin
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
