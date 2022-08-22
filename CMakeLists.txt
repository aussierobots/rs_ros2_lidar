cmake_minimum_required(VERSION 3.5)
project(rs_ros2_lidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost COMPONENTS system REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time regex)
find_package(rs_driver REQUIRED)

include_directories(include SYSTEM ${PCL_COMMON_INCLUDE_DIRS} ${rs_driver_INCLUDE_DIRS})

add_library(rs_lidar_components SHARED
  src/lidar_pub.cpp
  src/point_cloud_render.cpp
  src/lidar_viz.cpp
)
ament_generate_version_header(rs_lidar_components)

target_compile_definitions(rs_lidar_components
  PRIVATE "RS_LIDAR_BUILDING_DLL")

ament_target_dependencies(rs_lidar_components
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
)

target_link_libraries(rs_lidar_components
  ${rs_driver_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(rs_lidar_components PLUGIN "rs_lidar::LIDARPub" EXECUTABLE lidar_pub)
rclcpp_components_register_node(rs_lidar_components PLUGIN "rs_lidar::LIDARViz" EXECUTABLE lidar_viz)


install(TARGETS
  rs_lidar_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
