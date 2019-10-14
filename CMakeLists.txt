cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(physxtest LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall -fsized-deallocation")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall -fsized-deallocation")

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)

include_directories("${CMAKE_SOURCE_DIR}/../PhysX/physx/include"
        "${CMAKE_SOURCE_DIR}/../PhysX/pxshared/include" "3rd_party/optifuser/include"
        "src/renderer"
        )
link_directories("${CMAKE_SOURCE_DIR}/../PhysX/physx/bin/linux.clang/checked"
        "3rd_party/optifuser/build/lib"
        )

include_directories(
        "3rd_party/optifuser/include"
        "src"
        "src/urdf"
)

include_directories("3rd_party/optifuser/3rd_party/imgui" "3rd_party/optifuser/3rd_party/imgui/examples")

file(GLOB GUI_SRC "3rd_party/imgui/*.cpp"
        "3rd_party/imgui/examples/imgui_impl_glfw.cpp"
        "3rd_party/imgui/examples/imgui_impl_opengl3.cpp")


add_definitions(-D_DEBUG)
# add_definitions(-D_VERBOSE)
add_definitions(-D_PVD)
add_definitions(-DPVD_HOST="192.168.0.110")

add_subdirectory("3rd_party/optifuser")
add_subdirectory("3rd_party/pybind11")
include_directories("3rd_party/pybind11/include")

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")

add_library(mylib ${PROJECT_SRC})
target_link_libraries(mylib -Wl,--start-group
        libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
        libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
        libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a libPhysX_static_64.a
        libPhysXVehicle_static_64.a libSnippetRender_static_64.a
        libSnippetUtils_static_64.a -Wl,--end-group pthread ${OPENGL_LIBRARY} glfw
        GLEW ${ASSIMP_LIBRARIES} dl optifuser stdc++fs tinyxml2)

set_target_properties(mylib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_executable(run "test/main.cpp")
target_link_libraries(run mylib)

add_executable(ps3 test/ps3.cpp)
target_link_libraries(ps3 mylib)

pybind11_add_module(sapyen "python/pybind.cpp")
target_link_libraries(sapyen PRIVATE mylib)

add_executable(ant test/ant.cpp)
target_link_libraries(ant mylib)

add_executable(cheetah test/cheetah.cpp)
target_link_libraries(cheetah mylib)

# Below are the ROS specified content
set(SUPPORTED_ROS_VERSION "melodic" "kinetic" "lunar")
set(OLD_ROS_VERSION "indigo" "jade" "hydro")
set(ROS $ENV{ROS_DISTRO})
if (ROS)
    list(FIND SUPPORTED_ROS_VERSION ${ROS} _index)
    if (_index GREATER -1)
        message("Your ROS Distribution is ${ROS}")
        find_package(catkin REQUIRED COMPONENTS
                sapien_ros_utils
                roslaunch
                moveit_core
                moveit_ros_planning
                actionlib
                actionlib_msgs
                roscpp)

        include_directories(${catkin_INCLUDE_DIRS} "robot")

        add_library(robot_lib
                robot/controller/joint_trajectory_controller.cpp
                robot/controller/cartesian_velocity_controller.cpp
                robot/controller/velocity_control_service.cpp
                robot/controller/joint_pub_node.cpp
                robot/controller/controller_manger.cpp
                robot/device/joystick_ps3.cpp
                robot/device/InputDevice.cpp
                robot/device/movo_ps3.cpp
                )
        target_link_libraries(robot_lib mylib ${catkin_LIBRARIES})
        add_executable(robot robot/test/robot.cpp)
        target_link_libraries(robot robot_lib)
        add_executable(grasp robot/test/grasp.cpp)
        target_link_libraries(grasp robot_lib)
        add_executable(ps3_control robot/example/ps3_controller.cpp)
        target_link_libraries(ps3_control robot_lib)
    elseif (${ROS} IN_LIST OLD_ROS_VERSION)
        message("Your ROS version not supported. Please update to kinetic or above.")
    endif ()
else ()
    message(WARNING "No ROS Found")
endif ()
