add_executable(demo main.cpp)
add_executable(bgl_demo bgl_demo.cpp)
add_executable(ros_tutorial_publisher ros_tutorial_publisher.cpp)
add_executable(ros_tutorial_subscriber ros_tutorial_subscriber.cpp)
add_executable(ros_tutorial_service_server ros_tutorial_service_server.cpp)
add_executable(ros_tutorial_service_client ros_tutorial_service_client.cpp)
add_executable(rviz_client_demo rviz_client_demo.cpp)

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
        rviz_visual_tools
        moveit_ros_planning)

ament_target_dependencies(ros_tutorial_publisher
        rclcpp
        std_msgs
)

ament_target_dependencies(ros_tutorial_subscriber
        rclcpp
        std_msgs
        )

ament_target_dependencies(ros_tutorial_service_server
        rclcpp
        rcl_interfaces
        example_interfaces)

ament_target_dependencies(ros_tutorial_service_client
        rclcpp
        rcl_interfaces
        example_interfaces)

target_include_directories(rviz_client_demo
        PRIVATE
        ${RVIZ_CLIENT_INCLUDE}
        ${RF_COMMON_INCLUDE})
target_link_libraries(rviz_client_demo
        rviz_client)