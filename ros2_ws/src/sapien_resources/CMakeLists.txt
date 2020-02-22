cmake_minimum_required(VERSION 3.5)
project(sapien_resources)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY xarm6_description
        DESTINATION "share/${PROJECT_NAME}")

install(DIRECTORY xarm6_moveit_config DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY panda_urdf_description DESTINATION "share/${PROJECT_NAME}")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
