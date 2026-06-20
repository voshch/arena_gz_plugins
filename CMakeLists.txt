cmake_minimum_required(VERSION 3.14)
project(arena_gz_plugins)

find_package(ament_cmake REQUIRED)

# gz-sim 8 (Harmonic)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# gz-math 7 (Pose3d / Quaterniond)
find_package(gz-math7 REQUIRED)

# sdformat 14 (sdf::Root / sdf::Actor, for runtime actor creation)
find_package(sdformat14 REQUIRED)

# gz-gui 8 / gz-rendering 8 / Qt 5 (ViewportCamera GUI plugin)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

# ROS 2
find_package(rclcpp REQUIRED)
find_package(arena_people_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(arena_runtime_msgs REQUIRED)

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
  sdformat14::sdformat14
  rclcpp::rclcpp
  ${arena_people_msgs_TARGETS}
)

install(TARGETS PedSkeletonPlugin
  LIBRARY DESTINATION lib
)

# GUI plugin: moc the Q_OBJECT header explicitly so its meta-object, vtable and
# typeinfo are emitted (gz dlopens the plugin and resolves its typeinfo); qrc embeds
# the card QML.
qt5_wrap_cpp(VIEWPORT_CAMERA_MOC include/arena_gz_plugins/ViewportCamera.hh)
qt5_add_resources(VIEWPORT_CAMERA_RCC src/ViewportCamera.qrc)
add_library(ViewportCamera SHARED
  src/ViewportCamera.cc
  ${VIEWPORT_CAMERA_MOC}
  ${VIEWPORT_CAMERA_RCC}
)
target_include_directories(ViewportCamera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ViewportCamera PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-gui8::gz-gui8
  gz-rendering8::gz-rendering8
  gz-math7::gz-math7
  Qt5::Core
  Qt5::Quick
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${arena_runtime_msgs_TARGETS}
)
install(TARGETS ViewportCamera
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
