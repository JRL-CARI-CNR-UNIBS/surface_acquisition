cmake_minimum_required(VERSION 3.12)
project(surface_acquisition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features search kdtree segmentation)
find_package(OpenMP)

# ROS-independent core: crop -> voxel grid -> SOR -> clustering -> normals
add_library(cloud_filter_core SHARED
  src/cloud_filter.cpp
)
target_compile_features(cloud_filter_core PUBLIC cxx_std_17)
target_include_directories(cloud_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(cloud_filter_core PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(cloud_filter_core ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(cloud_filter_core OpenMP::OpenMP_CXX)
endif()

add_executable(cloud_filter_node
  src/cloud_filter_node.cpp
  src/cloud_filter_main.cpp
)
target_include_directories(cloud_filter_node PRIVATE ${pcl_conversions_INCLUDE_DIRS})
target_link_libraries(cloud_filter_node
  cloud_filter_core
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${pcl_conversions_TARGETS}
)

install(TARGETS cloud_filter_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS cloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/fake_realsense.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cloud_filter test/test_cloud_filter.cpp)
  target_link_libraries(test_cloud_filter cloud_filter_core)
endif()

ament_package()
