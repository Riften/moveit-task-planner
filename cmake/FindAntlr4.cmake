set( Antlr4_FOUND FALSE )

message(STATUS "Searching for antlr4-runtime/antlr4-runtime.h")
find_path( Antlr4_INCLUDE_DIR antlr4-runtime/antlr4-runtime.h )

message(STATUS "Searching for libantlr4-runtime")
find_library( Antlr4_LIBRARY antlr4-runtime )

include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( Antlr4 Antlr4_INCLUDE_DIR Antlr4_LIBRARY )
if( ANTLR4_FOUND )
    set( Antlr4_FOUND TRUE )
endif()
