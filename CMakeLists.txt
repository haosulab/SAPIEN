cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
project(physxtest LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall")

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL)
find_package(GLEW)
find_package(glfw3)

include_directories("/home/fx/source/PhysX/physx/include" "/home/fx/source/PhysX/pxshared/include" "/home/fx/optifuser/include")
link_directories("/home/fx/source/PhysX/physx/bin/linux.clang/debug" "/home/fx/optifuser/build/lib")


add_definitions(-D_DEBUG)

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")

add_executable(run ${PROJECT_SRC})
target_link_libraries(run libphysxrepack.a pthread dl GL GLU GLEW glfw optifuser)

add_executable(test "src/test.cpp")
target_link_libraries(test libphysxrepack.a pthread dl GL GLU GLEW glfw optifuser)
