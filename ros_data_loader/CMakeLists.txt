SET(LIB_NAME "ros_data_loader")
add_library(${LIB_NAME}
        src/ros_data_loader.cpp
        src/moveit_data_loader.cpp)
target_link_libraries(${LIB_NAME} ${Log4cxx_LIBRARY})
target_link_libraries(${LIB_NAME} ${YAML_CPP_LIBRARIES} Boost::filesystem)
ament_target_dependencies(${LIB_NAME} rclcpp ament_index_cpp)
target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${RF_COMMON_INCLUDE}>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${Log4cxx_INCLUDE_DIR}>
        )
