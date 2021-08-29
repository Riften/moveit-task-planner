cmake_minimum_required(VERSION 3.5)
project(moveit_task_planner)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Log4cxx REQUIRED)
find_package(ament_index_cpp REQUIRED)
# MESSAGE(STATUS "Found moveit_core_INCLUDE_DIRS ${moveit_core_INCLUDE_DIRS}")
# find_package(moveit_ros_planning REQUIRED)
# MESSAGE(STATUS "Found moveit_ros_planning_INCLUDE_DIRS ${moveit_ros_planning_INCLUDE_DIRS}")

SET(RF_COMMON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/robotflow_common")
SET(ROS_DATA_LOADER_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/ros_data_loader/include")

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

add_subdirectory(ros_data_loader)
add_subdirectory(demo)

ament_package()
