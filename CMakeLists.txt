cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
project(physxtest LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall")

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

include_directories("/home/fx/source/PhysX/physx/include" "/home/fx/source/PhysX/pxshared/include" "/home/fx/optifuser/include" "src")
link_directories("/home/fx/source/PhysX/physx/bin/linux.clang/debug" "/home/fx/optifuser/build/lib")

add_definitions(-D_DEBUG)

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")

add_library(mylib ${PROJECT_SRC})
target_link_libraries(mylib libphysxrepack.a pthread ${OPENGL_LIBRARY} glfw ${GLEW_LIBRARY}
  ${ASSIMP_LIBRARIES} dl optifuser stdc++fs tinyxml2)

add_executable(run "test/main.cpp")
target_link_libraries(run mylib)

add_executable(test "test/test.cpp")
target_link_libraries(test mylib)
