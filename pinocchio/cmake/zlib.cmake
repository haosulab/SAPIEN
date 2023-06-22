if(TARGET zlibstatic)
    return()
endif()

include(FetchContent)

FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG        v1.2.11
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(zlib)
set_target_properties(zlibstatic PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set(ZLIB_INCLUDE_DIR ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
set(ZLIB_LIBRARY zlibstatic)
add_library(ZLIB::ZLIB ALIAS zlibstatic)
