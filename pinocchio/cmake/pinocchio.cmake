set(BUILD_TESTING OFF)
set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_COLLISION_SUPPORT OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_AUTODIFF_SUPPORT OFF)
set(BUILD_WITH_URDF_SUPPORT ON)
set(Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    tinyxml
    URL       https://phoenixnap.dl.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.tar.gz
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
    GIT_TAG        v2.6.18
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

FetchContent_Declare(
    boost
    URL      https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.81.0.tar.gz
    URL_HASH MD5=ffac94fbdd92d6bc70a897052022eeba
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(boost)
FetchContent_MakeAvailable(tinyxml)
FetchContent_MakeAvailable(urdfdom_headers)
FetchContent_MakeAvailable(urdfdom)
FetchContent_MakeAvailable(pinocchio)

set_target_properties(boost_filesystem PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(boost_serialization PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(boost_system PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(boost_thread PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(urdfdom_headers INTERFACE $<BUILD_INTERFACE:${urdfdom_headers_SOURCE_DIR}/include>)
target_include_directories(console_bridge PUBLIC $<BUILD_INTERFACE:${console_bridge_BINARY_DIR}>)
target_compile_definitions(pinocchio PUBLIC PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
target_compile_definitions(pinocchio PUBLIC PINOCCHIO_WITH_URDFDOM)
target_link_libraries(urdfdom_model PUBLIC urdfdom_headers tinyxml)
target_link_libraries(pinocchio PUBLIC urdfdom_headers urdfdom_model console_bridge eigen)
target_link_libraries(pinocchio PUBLIC
    Boost::math Boost::filesystem Boost::serialization Boost::system
    Boost::asio Boost::iostreams Boost::foreach Boost::format Boost::property_tree
)
target_include_directories(pinocchio PUBLIC
    # ${boost_SOURCE_DIR}/libs/tuple/include
    # ${boost_SOURCE_DIR}/libs/optional/include
    # ${boost_SOURCE_DIR}/libs/system/include
    # ${boost_SOURCE_DIR}/libs/filesystem/include
    # ${boost_SOURCE_DIR}/libs/concept_check/include
    # ${boost_SOURCE_DIR}/libs/iostreams/include
    # ${boost_SOURCE_DIR}/libs/align/include
    # ${boost_SOURCE_DIR}/libs/asio/include
    # ${boost_SOURCE_DIR}/libs/bind/include
    # ${boost_SOURCE_DIR}/libs/iterator/include
    # ${boost_SOURCE_DIR}/libs/range/include
    # ${boost_SOURCE_DIR}/libs/smart_ptr/include
    # ${boost_SOURCE_DIR}/libs/io/include
    # ${boost_SOURCE_DIR}/libs/function_types/include
    # ${boost_SOURCE_DIR}/libs/fusion/include
    # ${boost_SOURCE_DIR}/libs/serialization/include
    # ${boost_SOURCE_DIR}/libs/integer/include
    # ${boost_SOURCE_DIR}/libs/move/include
    # ${boost_SOURCE_DIR}/libs/utility/include
    # ${boost_SOURCE_DIR}/libs/preprocessor/include
    # ${boost_SOURCE_DIR}/libs/detail/include
    # ${boost_SOURCE_DIR}/libs/mpl/include
    # ${boost_SOURCE_DIR}/libs/core/include
    # ${boost_SOURCE_DIR}/libs/assert/include
    # ${boost_SOURCE_DIR}/libs/throw_exception/include
    # ${boost_SOURCE_DIR}/libs/container_hash/include
    # ${boost_SOURCE_DIR}/libs/type_index/include
    # ${boost_SOURCE_DIR}/libs/variant/include
    # ${boost_SOURCE_DIR}/libs/static_assert/include
    # ${boost_SOURCE_DIR}/libs/config/include
    # ${boost_SOURCE_DIR}/libs/type_traits/include
    # ${boost_SOURCE_DIR}/libs/math/include
    # ${boost_SOURCE_DIR}/libs/foreach/include
    # ${boost_SOURCE_DIR}/libs/format/include
    # ${boost_SOURCE_DIR}/libs/property_tree/include
    # ${boost_SOURCE_DIR}/libs/any/include
    # ${boost_SOURCE_DIR}/libs/multi_index/include
)
target_include_directories(pinocchio INTERFACE ${pinocchio_BINARY_DIR}/include)

set_target_properties(tinyxml PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(console_bridge PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(urdfdom_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(pinocchio PROPERTIES POSITION_INDEPENDENT_CODE ON)
