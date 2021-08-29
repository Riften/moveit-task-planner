add_executable(demo main.cpp)

target_include_directories(demo
        PRIVATE
        ${RF_COMMON_INCLUDE}
        ${Log4cxx_INCLUDE_DIR}
        ${ROS_DATA_LOADER_INCLUDE})

target_link_libraries(demo ${Log4cxx_LIBRARY} ros_data_loader)

ament_target_dependencies(demo moveit_ros_planning rclcpp)