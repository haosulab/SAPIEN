if(TARGET pinocchio)
    return()
endif()

set(BUILD_TESTING OFF)
set(BUILD_PYTHON_INTERFACE OFF)
set(BUILD_WITH_AUTODIFF_SUPPORT OFF)
set(BUILD_WITH_URDF_SUPPORT ON)
set(Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    urdfdom_headers
    GIT_REPOSITORY https://github.com/ros/urdfdom_headers.git
    GIT_TAG        1.1.0
    PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_headers_CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt
    OVERRIDE_FIND_PACKAGE
)
set(urdfdom_headers_VERSION 1.1.0 CACHE STRING "" FORCE)  # make pinocchio believe

FetchContent_Declare(
    console_bridge
    GIT_REPOSITORY https://github.com/ros/console_bridge.git
    GIT_TAG        1.0.2
    OVERRIDE_FIND_PACKAGE
)

FetchContent_Declare(
    urdfdom
    GIT_REPOSITORY https://github.com/ros/urdfdom.git
    PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt && cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_parser_CMakeLists.txt <SOURCE_DIR>/urdf_parser/CMakeLists.txt
    GIT_TAG        3.1.1
    OVERRIDE_FIND_PACKAGE
)

FetchContent_Declare(
    pinocchio
    GIT_REPOSITORY https://github.com/stack-of-tasks/pinocchio.git
    PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pinocchio_post-project.cmake <SOURCE_DIR>/cmake/post-project.cmake
    GIT_TAG        v2.5.6
)

FetchContent_MakeAvailable(urdfdom_headers)
FetchContent_MakeAvailable(urdfdom)
FetchContent_MakeAvailable(pinocchio)

target_include_directories(urdfdom_headers INTERFACE $<BUILD_INTERFACE:${urdfdom_headers_SOURCE_DIR}/include>)
target_include_directories(console_bridge PUBLIC $<BUILD_INTERFACE:${console_bridge_BINARY_DIR}>)
target_compile_definitions(pinocchio PUBLIC PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
target_compile_definitions(pinocchio PUBLIC PINOCCHIO_WITH_URDFDOM)
target_link_libraries(urdfdom_model PUBLIC urdfdom_headers)
target_link_libraries(pinocchio PUBLIC urdfdom_headers urdfdom_model console_bridge eigen)

set_target_properties(console_bridge PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(urdfdom_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(pinocchio PROPERTIES POSITION_INDEPENDENT_CODE ON)
