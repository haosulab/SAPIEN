cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(sapien LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall -fsized-deallocation")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall -fsized-deallocation")

set(CMAKE_DEBUG_POSTFIX "")
set(CMAKE_BUILD_TYPE "Debug")
set(OpenGL_GL_PREFERENCE "LEGACY")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("-- DEBUG Build")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})
else ()
    message("-- RELEASE Build")
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_RELEASE})
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(spdlog REQUIRED)

include_directories("${CMAKE_SOURCE_DIR}/../PhysX/physx/include"
        "${CMAKE_SOURCE_DIR}/../PhysX/pxshared/include" "3rd_party/optifuser/include"
        "src/renderer"
        )

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    link_directories("${CMAKE_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/checked"
            "3rd_party/optifuser/build/lib")
else ()
    link_directories("${CMAKE_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/release"
            "3rd_party/optifuser/build/lib")
endif ()

include_directories(
        "3rd_party/optifuser/include"
        "src"
        "ros2_src"
        "unittest"
)

include_directories("3rd_party/optifuser/3rd_party/imgui" "3rd_party/optifuser/3rd_party/imgui/examples")

include_directories("3rd_party/v-hacd/src/VHACD_Lib/public")

file(GLOB GUI_SRC "3rd_party/imgui/*.cpp"
        "3rd_party/imgui/examples/imgui_impl_glfw.cpp"
        "3rd_party/imgui/examples/imgui_impl_opengl3.cpp")

set(OPTIX_HOME "NOT FOUND" CACHE FILEPATH "Home to OptiX")
if (EXISTS ${OPTIX_HOME})
    include_directories("${OPTIX_HOME}/include")
    link_directories("${OPTIX_HOME}/lib64")
    set(OPTIX_LIBRARY optix)
    add_definitions(-D_USE_OPTIX)
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-D_DEBUG)
    # add_definitions(-D_PVD)
    add_definitions(-DPVD_HOST="192.168.0.110")
else ()
    add_definitions(-DNDEBUG)
endif ()

add_subdirectory("3rd_party/optifuser")
add_subdirectory("3rd_party/pybind11")
add_subdirectory("3rd_party/tinyxml2")
add_subdirectory("3rd_party/v-hacd/src")
set_target_properties(vhacd PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

include_directories("3rd_party/pybind11/include")

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")
add_custom_target(shader COMMAND cp -r ${CMAKE_CURRENT_SOURCE_DIR}/glsl_shader ${CMAKE_CURRENT_BINARY_DIR})
add_library(sapien ${PROJECT_SRC})
add_dependencies(sapien shader)

if (EXISTS ${OPTIX_HOME})
    add_dependencies(sapien ptx_dir)
endif ()

target_link_libraries(sapien -Wl,--start-group
        libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
        libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
        libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a
        libPhysX_static_64.a libPhysXVehicle_static_64.a
        libSnippetRender_static_64.a libSnippetUtils_static_64.a -Wl,--end-group
        pthread ${OPENGL_LIBRARY} glfw GLEW ${ASSIMP_LIBRARIES} dl optifuser vhacd
        stdc++fs tinyxml2 ${OPTIX_LIBRARY} ${SPDLOG_LIBRARIES})

set_target_properties(optifuser PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(sapien PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# pybind11_add_module(sapyen "python/pybind.cpp")
# target_link_libraries(sapyen PRIVATE sapien)

pybind11_add_module(pysapien "python/pysapien.cpp")
target_link_libraries(pysapien PRIVATE sapien)

add_executable(creation unittest/creation.cpp)
target_link_libraries(creation sapien)

add_executable(manual_actor manualtest/actor.cpp)
target_link_libraries(manual_actor sapien)
add_executable(manual_articulation manualtest/articulation.cpp)
target_link_libraries(manual_articulation sapien)
add_executable(manual_urdf manualtest/urdf.cpp)
target_link_libraries(manual_urdf sapien)
add_executable(manual_kinematics manualtest/kinematics.cpp)
target_link_libraries(manual_kinematics sapien)

add_custom_target(python_test COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/test/*.py ${CMAKE_CURRENT_SOURCE_DIR}/test/*.json ${CMAKE_CURRENT_BINARY_DIR})

add_custom_target(manual_python COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/manualtest/*.py ${CMAKE_CURRENT_BINARY_DIR})


# Below are the ROS2 specified cmake
set(SUPPORTED_ROS_VERSION "dashing" "eloquent")
set(OLD_ROS_VERSION "crystal" "bouncy" "ardent")
set(ROS $ENV{ROS_DISTRO})
if (ROS)
    list(FIND SUPPORTED_ROS_VERSION ${ROS} _index)
    if (_index GREATER -1)
        message("Your ROS Distribution is ${ROS}")
        if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            add_compile_options(-Wall -Wextra -Wpedantic)
        endif ()
        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sapien_ros2_communication_interface REQUIRED)
        find_package(moveit_core)
        find_package(moveit_ros_planning)
        find_package(ament_index_cpp)

        include_directories(
                "/opt/ros/${ROS}/include"
                "ros2_src"
        )

        file(GLOB_RECURSE SAPIEN_URDF_SRC "ros2_src/urdf/*.cpp" "ros2_src/urdf/*.h")
        add_library(sapien_ros_urdf ${SAPIEN_URDF_SRC} ros2_src/test/test_ros_urdf.cpp)
        target_link_libraries(sapien_ros_urdf tinyxml2 sapien)
        ament_target_dependencies(sapien_ros_urdf rclcpp ament_index_cpp)

        file(GLOB_RECURSE SAPIEN_DEVICE_SRC "ros2_src/device/*.cpp" "ros2_src/device/*.h")
        add_library(sapien_device ${SAPIEN_DEVICE_SRC})
        ament_target_dependencies(sapien_device rclcpp sensor_msgs)

        file(GLOB_RECURSE SAPIEN_ROBOT_SRC "ros2_src/controller/*.cpp" "ros2_src/articulation/*.cpp" "ros2_src/utils/*.hpp")
        add_library(sapien_controller ${SAPIEN_ROBOT_SRC})
        target_link_libraries(sapien_controller sapien)
        ament_target_dependencies(sapien_controller rclcpp std_msgs sensor_msgs std_msgs sapien_ros2_communication_interface moveit_core moveit_ros_planning)

        # test
        add_executable(manual_PS3 manualtest/PS3ROSTest.cpp)
        target_link_libraries(manual_PS3 sapien_device)

        add_executable(test_ros2 ros2_src/test/test_sapien_ros2.cpp)
        target_link_libraries(test_ros2 sapien_controller sapien_device)

        add_executable(test_ros_urdf ros2_src/test/test_ros_urdf.cpp)
        target_link_libraries(test_ros_urdf sapien_ros_urdf)

        ament_package()
    elseif (${ROS} IN_LIST OLD_ROS_VERSION)
        message("Your ROS version not supported. Please update to kinetic or above.")
    endif ()
else ()
    message(WARNING "No ROS Found, will not compile source in ros2_src")
endif ()
