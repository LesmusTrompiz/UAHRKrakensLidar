cmake_minimum_required(VERSION 3.5)
project(uahrkrakens_lidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -o3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Add directories
set(UTILS_PATH "./include/uahrkrakens_lidar")

FILE(GLOB UTILS_SRC 
  "${UTILS_PATH}/*.cpp"
)

include_directories(
  ${UTILS_PATH}
)


add_executable(analize_scan src/analize_scan_node.cpp ${UTILS_SRC})
target_link_libraries(analize_scan)

ament_target_dependencies(analize_scan rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  find_package(ros_testing REQUIRED)

  ament_add_gtest(get_clusters_test tests/GetClusters.cpp ${UTILS_SRC})
  target_include_directories(get_clusters_test PUBLIC tests/) 
  
  ament_add_gtest(laser_range_to_2d_points_test tests/LaserRangeTo2dPoints.cpp ${UTILS_SRC})
  target_include_directories(laser_range_to_2d_points_test PUBLIC tests/) 

  ament_add_gtest(get_clusters_length_test tests/GetClusterLength.cpp ${UTILS_SRC})
  target_include_directories(get_clusters_length_test PUBLIC tests/) 

endif()

ament_package()
