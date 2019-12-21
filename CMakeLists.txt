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
        "src/urdf"
        "unittest"
)

include_directories("3rd_party/optifuser/3rd_party/imgui" "3rd_party/optifuser/3rd_party/imgui/examples")

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
    add_definitions(-D_PVD)
    add_definitions(-DPVD_HOST="192.168.0.110")
else ()
    add_definitions(-DNDEBUG)
endif ()

add_subdirectory("3rd_party/optifuser")
add_subdirectory("3rd_party/pybind11")
add_subdirectory("3rd_party/tinyxml2")

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
        pthread ${OPENGL_LIBRARY} glfw GLEW ${ASSIMP_LIBRARIES} dl optifuser
        stdc++fs tinyxml2 ${OPTIX_LIBRARY} ${SPDLOG_LIBRARIES} )

set_target_properties(optifuser PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(sapien PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_executable(run "test/main.cpp")
target_link_libraries(run sapien)

add_executable(test_kinematic "test/test_kinematic.cpp")
target_link_libraries(test_kinematic sapien)

add_executable(ps3 test/ps3.cpp)
target_link_libraries(ps3 sapien)

pybind11_add_module(sapyen "python/pybind.cpp")
target_link_libraries(sapyen PRIVATE sapien)

add_executable(cheetah test/cheetah.cpp)
target_link_libraries(cheetah sapien)

add_executable(ant test/ant.cpp)
target_link_libraries(ant sapien)

add_executable(creation unittest/creation.cpp)
target_link_libraries(creation sapien)

add_custom_target(python_test COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/test/*.py ${CMAKE_CURRENT_SOURCE_DIR}/test/*.json ${CMAKE_CURRENT_BINARY_DIR})

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
                moveit_ros_planning_interface
                actionlib
                actionlib_msgs
                roscpp)

        include_directories(${catkin_INCLUDE_DIRS} "robot")

        add_library(sapien_robot
                robot/controller/joint_trajectory_controller.cpp
                robot/controller/cartesian_velocity_controller.cpp
                robot/controller/velocity_control_service.cpp
                robot/controller/joint_pub_node.cpp
                robot/controller/controller_manger.cpp
                robot/controller/group_planner.cpp
                robot/device/input_device.cpp
                robot/device/joystick_ps3.cpp
                )
        target_link_libraries(sapien_robot sapien ${catkin_LIBRARIES})

        add_executable(robot robot/test/robot.cpp)
        target_link_libraries(robot sapien_robot)
        add_executable(movo_ps3 robot/example/movo_ps3_test.cpp)
        target_link_libraries(movo_ps3 sapien_robot)
        add_executable(gripper_ps3 robot/example/single_gripper_ps3_test.cpp)
        target_link_libraries(gripper_ps3 sapien_robot)
        #        add_executable(movo_keyboard robot/example/movo_keyboard_test.cpp)
        #        target_link_libraries(movo_keyboard sapien_robot)
        pybind11_add_module(sapyen_robot "python/pybind-robot.cpp")
        target_link_libraries(sapyen_robot PRIVATE sapien_robot)
        add_executable(xarm_ps3 robot/example/xarm_ps3_test.cpp)
        target_link_libraries(xarm_ps3 sapien_robot)
        add_executable(paper_demo robot/example/paper_demo.cpp)
        target_link_libraries(paper_demo sapien_robot)

        set_target_properties(sapien_robot PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
        set_target_properties(sapien_robot PROPERTIES POSITION_INDEPENDENT_CODE TRUE)


    elseif (${ROS} IN_LIST OLD_ROS_VERSION)
        message("Your ROS version not supported. Please update to kinetic or above.")
    endif ()
else ()
    message(WARNING "No ROS Found")
endif ()
