SET(LIB_NAME bullet_client)
add_library(${LIB_NAME} src/bullet_client.cpp)
target_link_libraries(${LIB_NAME}
        ${Log4cxx_LIBRARY}

        )
target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        #$<BUILD_INTERFACE:${RF_COMMON_INCLUDE}>
        $<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/examples>
        $<BUILD_INTERFACE:${BULLET_PHYSICS_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${Log4cxx_INCLUDE_DIR}>)