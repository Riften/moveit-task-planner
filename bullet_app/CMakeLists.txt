SET(LIB_NAME bullet_app)
add_library(${LIB_NAME} src/bullet_app.cpp)
target_link_libraries(${LIB_NAME} ${Log4cxx_LIBRARY})
target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        #$<BUILD_INTERFACE:${RF_COMMON_INCLUDE}>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${Log4cxx_INCLUDE_DIR}>)