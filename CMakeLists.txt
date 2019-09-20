cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(physxtest LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall")

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM3 REQUIRED gtkmm-3.0)

include_directories("/home/fx/source/PhysX/physx/include"
  "/home/fx/source/PhysX/pxshared/include" "/home/fx/source/optifuser/include" "src"
  ${GTKMM3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/3rd_party/v-hacd/src/VHACD_Lib/inc
  )
link_directories("/home/fx/source/PhysX/physx/bin/linux.clang/debug"
  "/home/fx/source/optifuser/build/lib" ${GTKMM3_LIBRARY_DIRS}
  ${CMAKE_SOURCE_DIR}/3rd_party/v-hacd/src/VHACD_Lib/inc
  ${CMAKE_SOURCE_DIR}/3rd_party/v-hacd/build/linux/VHACD_Lib
  )

add_definitions(-D_DEBUG)
add_definitions(-D_VERBOSE)
add_definitions(-D_PVD)
add_definitions(-DPVD_HOST="192.168.0.110")

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")

add_library(mylib ${PROJECT_SRC})
target_link_libraries(mylib -Wl,--start-group
  libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
  libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
  libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a libPhysX_static_64.a
  libPhysXVehicle_static_64.a libSnippetRender_static_64.a
  libSnippetUtils_static_64.a -Wl,--end-group pthread ${OPENGL_LIBRARY} glfw
  libvhacd.a ${GLEW_LIBRARY} ${ASSIMP_LIBRARIES} dl optifuser stdc++fs tinyxml2)

add_executable(sample "test/sample.cpp")
target_link_libraries(sample mylib glut)

add_executable(run "test/main.cpp")
target_link_libraries(run mylib)

add_executable(gtk "test/gtk.cpp")
target_link_libraries(gtk mylib ${GTKMM3_LIBRARIES})

add_executable(test "test/test.cpp")
target_link_libraries(test mylib)
