if(TARGET pinocchio)
    return()
endif()

set(BUILD_TESTING OFF)
set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_COLLISION_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_AUTODIFF_SUPPORT OFF)
set(BUILD_WITH_URDF_SUPPORT ON)
set(Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    tinyxml
    URL       https://cytranet.dl.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.tar.gz
    URL_HASH  MD5=c1b864c96804a10526540c664ade67f0
    PATCH_COMMAND cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/tinyxml_CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt
)

FetchContent_Declare(
    urdfdom_headers
    GIT_REPOSITORY https://github.com/ros/urdfdom_headers.git
    GIT_TAG        1.1.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_headers_CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt
    OVERRIDE_FIND_PACKAGE
)
set(urdfdom_headers_VERSION 1.1.0 CACHE STRING "" FORCE)  # make pinocchio believe

FetchContent_Declare(
    console_bridge
    GIT_REPOSITORY https://github.com/ros/console_bridge.git
    GIT_TAG        1.0.2
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    OVERRIDE_FIND_PACKAGE
)

FetchContent_Declare(
    urdfdom
    GIT_REPOSITORY https://github.com/ros/urdfdom.git
    PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt && cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/urdfdom_parser_CMakeLists.txt <SOURCE_DIR>/urdf_parser/CMakeLists.txt
    GIT_TAG        3.1.1
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    OVERRIDE_FIND_PACKAGE
)

FetchContent_Declare(
    pinocchio
    GIT_REPOSITORY https://github.com/stack-of-tasks/pinocchio.git
    PATCH_COMMAND  cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pinocchio_post-project.cmake <SOURCE_DIR>/cmake/post-project.cmake && cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pinocchio_CMakeLists.txt <SOURCE_DIR>/CMakeLists.txt && cmake -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pinocchio_src_CMakeLists.txt <SOURCE_DIR>/src/CMakeLists.txt
    GIT_TAG        v2.5.6
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(tinyxml)
FetchContent_MakeAvailable(urdfdom_headers)
FetchContent_MakeAvailable(urdfdom)
FetchContent_MakeAvailable(pinocchio)

target_include_directories(urdfdom_headers INTERFACE $<BUILD_INTERFACE:${urdfdom_headers_SOURCE_DIR}/include>)
target_include_directories(console_bridge PUBLIC $<BUILD_INTERFACE:${console_bridge_BINARY_DIR}>)
target_compile_definitions(pinocchio PUBLIC PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
target_compile_definitions(pinocchio PUBLIC PINOCCHIO_WITH_URDFDOM)
target_link_libraries(urdfdom_model PUBLIC urdfdom_headers tinyxml)
target_link_libraries(pinocchio PUBLIC urdfdom_headers urdfdom_model console_bridge eigen)
target_link_libraries(pinocchio PUBLIC Boost::math Boost::filesystem Boost::serialization Boost::system)
target_include_directories(pinocchio PUBLIC
    ${boost_SOURCE_DIR}/libs/foreach/include
    ${boost_SOURCE_DIR}/libs/format/include
    ${boost_SOURCE_DIR}/libs/property_tree/include
    ${boost_SOURCE_DIR}/libs/any/include
    ${boost_SOURCE_DIR}/libs/multi_index/include
)
target_include_directories(pinocchio INTERFACE ${pinocchio_BINARY_DIR}/include)

set_target_properties(tinyxml PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(console_bridge PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(urdfdom_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(pinocchio PROPERTIES POSITION_INDEPENDENT_CODE ON)
