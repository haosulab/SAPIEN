if(TARGET minizip)
    return()
endif()

set(MZ_LZMA OFF CACHE BOOL "" FORCE)
set(MZ_ZSTD OFF CACHE BOOL "" FORCE)

include(FetchContent)

FetchContent_Declare(
    minizip
    GIT_REPOSITORY https://github.com/zlib-ng/minizip-ng.git
    GIT_TAG        3.0.10
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailableExclude(minizip)
set_target_properties(minizip PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

if (zlib_SOURCE_DIR)
  target_include_directories(minizip PRIVATE ${zlib_SOURCE_DIR} ${zlib_BINARY_DIR})
endif()
