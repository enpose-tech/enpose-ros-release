cmake_minimum_required(VERSION 3.19)
project(enpose_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Pull in the Enpose API (built from its Rust source via cargo). The C++
# wrapper target carries both the headers and the compiled shared library.
include(FetchContent)
FetchContent_Declare(enpose_api
  GIT_REPOSITORY https://github.com/enpose-tech/enpose-api
  GIT_TAG 0.2.0)
FetchContent_MakeAvailable(enpose_api)

add_executable(enpose_node src/enpose_node.cpp)
# ament_target_dependencies() was removed in recent Rolling; link the modern
# CMake targets exported by each package directly. Message packages expose their
# typesupport libraries via the <pkg>_TARGETS variable.
target_link_libraries(enpose_node PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs)

# Link the Enpose API. We take the headers and the cargo build dependency from
# the upstream targets, but link the shared library by name (-L<dir> -lenpose_api)
# instead of by its full build-tree path. cargo emits libenpose_api.so without a
# DT_SONAME; linking it by full path (as the upstream imported target does) would
# bake that non-relocatable build-tree path into the node's NEEDED entry, so the
# loader could not find the library once installed. Linking by name records the
# bare "libenpose_api.so", which is then resolved from lib/ (ament puts the
# package's lib/ directory on LD_LIBRARY_PATH) at run time.
add_dependencies(enpose_node enpose_api_cargo)
target_include_directories(enpose_node PRIVATE
  "${enpose_api_SOURCE_DIR}/c/include"
  "${enpose_api_SOURCE_DIR}/cpp/include")
get_target_property(ENPOSE_SHARED_LIB enpose_api_shared IMPORTED_LOCATION)
get_filename_component(ENPOSE_LIB_DIR "${ENPOSE_SHARED_LIB}" DIRECTORY)
target_link_directories(enpose_node PRIVATE "${ENPOSE_LIB_DIR}")
target_link_libraries(enpose_node PRIVATE "-lenpose_api")

# The Enpose shared library lives in the FetchContent build tree and is not a
# system library, so install it alongside the node so it ships with the package.
install(FILES "${ENPOSE_SHARED_LIB}" DESTINATION lib)

install(TARGETS enpose_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
