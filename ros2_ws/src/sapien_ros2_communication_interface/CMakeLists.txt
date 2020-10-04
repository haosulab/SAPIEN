cmake_minimum_required(VERSION 3.5)
project(sapien_ros2_communication_interface)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(MSG_FILES
        ""
        )

set(SRV_FILES
        "srv/JointVelocity.srv"
        "srv/CartesianVelocity.srv"
        )
rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILES}
        ${SRV_FILES}
        DEPENDENCIES builtin_interfaces geometry_msgs
        )


ament_package()
