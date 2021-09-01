add_executable(demo main.cpp)

target_include_directories(demo
        PRIVATE
        ${RF_COMMON_INCLUDE}
        ${Log4cxx_INCLUDE_DIR}
        ${ROS_DATA_LOADER_INCLUDE}
        ${BULLET_APP_INCLUDE}
        ${BULLET_PHYSICS_SOURCE_DIR}/examples
        ${BULLET_PHYSICS_SOURCE_DIR}/src
        )

target_link_libraries(demo ${Log4cxx_LIBRARY} ros_data_loader bullet_app)

ament_target_dependencies(demo
        moveit_ros_planning
        rclcpp
        pluginlib
        moveit_ros_planning_interface
        moveit_ros_visualization
        rviz_visual_tools)