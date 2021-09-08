SET(LIB_NAME "rviz_client")
add_library(${LIB_NAME} src/rviz_client.cpp)

ament_target_dependencies(${LIB_NAME}
        rclcpp
        rcl_interfaces
        )

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${ROS_DATA_LOADER_INCLUDE}>
        $<BUILD_INTERFACE:${Log4cxx_INCLUDE_DIR}>
        )

target_link_libraries(${LIB_NAME}
        ${Log4cxx_LIBRARY}
        ros_data_loader)