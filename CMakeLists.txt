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
  add_compile_options(-Wall -Wextra -Wpedantic)
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

  ament_add_gtest(test_utils tests/scan_utils_test.cpp ${UTILS_SRC})
  target_include_directories(test_utils PUBLIC tests/) 

endif()

ament_package()
