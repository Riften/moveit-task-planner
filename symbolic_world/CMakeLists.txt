SET(LIB_NAME symbolic_world)

add_library(${LIB_NAME} src/symbolic_world.cpp)

ament_target_dependencies(${LIB_NAME} moveit_core)

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${Log4cxx_INCLUDE_DIR}>
        )