cmake_minimum_required(VERSION 3.14)
project(arena_gz_plugins)

find_package(ament_cmake REQUIRED)

# gz-sim 8 (Harmonic)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# gz-math 7 (Pose3d / Quaterniond)
find_package(gz-math7 REQUIRED)

# gz-common 5 graphics (skin skeleton + idle clip for per-bone actor driving)
find_package(gz-common5 REQUIRED COMPONENTS graphics)

# bone map JSON
find_package(nlohmann_json REQUIRED)

# character annotations (pelvis clearance)
find_package(yaml-cpp REQUIRED)

# sdformat 14 (sdf::Root / sdf::Actor, for runtime actor creation)
find_package(sdformat14 REQUIRED)

# gz-gui 8 / gz-rendering 8 / Qt 5 (ViewportCamera GUI plugin)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

# ROS 2
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(arena_people_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(viewport_control_msgs REQUIRED)

# seat and bed placement, gz-math only so it is testable without a simulator
add_library(arena_gz_support STATIC
  src/Support.cc
)
set_target_properties(arena_gz_support PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(arena_gz_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(arena_gz_support PUBLIC
  gz-math7::gz-math7
  yaml-cpp::yaml-cpp
)

add_library(PedSkeletonPlugin SHARED
  src/PedSkeletonPlugin.cc
)
target_include_directories(PedSkeletonPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(PedSkeletonPlugin PUBLIC
  arena_gz_support
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-math7::gz-math7
  gz-common5::graphics
  sdformat14::sdformat14
  nlohmann_json::nlohmann_json
  rclcpp::rclcpp
  ${arena_people_msgs_TARGETS}
)

# components::BoneTransforms comes from the gz-sim overlay (arena feature gazebo update)
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})
check_cxx_source_compiles("
#include <gz/sim/components/Actor.hh>
int main() { gz::sim::components::BoneTransforms bones; return 0; }
" GZ_SIM_HAS_BONE_TRANSFORMS)
unset(CMAKE_REQUIRED_LIBRARIES)
if(GZ_SIM_HAS_BONE_TRANSFORMS)
  target_compile_definitions(PedSkeletonPlugin PRIVATE ARENA_GZ_BONE_TRANSFORMS)
else()
  message(WARNING "gz-sim${GZ_SIM_VER} has no BoneTransforms component, PedSkeletonPlugin "
    "scrubs actor clips only. `arena feature gazebo update` builds the overlay.")
endif()

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
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${viewport_control_msgs_TARGETS}
)
install(TARGETS ViewportCamera
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_support test/test_support.cc)
  target_link_libraries(test_support arena_gz_support)
endif()

ament_package()
