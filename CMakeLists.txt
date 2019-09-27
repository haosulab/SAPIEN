cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(physxtest LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "-o0 -g3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-o3 -g0 -Wall")

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS_DEBUG})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)

include_directories("/home/fx/source/PhysX/physx/include"
  "/home/fx/source/PhysX/pxshared/include" "3rd_party/optifuser/include" "src"
  "src/renderer"
  )
link_directories("/home/fx/source/PhysX/physx/bin/linux.clang/debug"
  "3rd_party/optifuser/build/lib"
  )


include_directories("3rd_party/optifuser/3rd_party/imgui" "3rd_party/optifuser/3rd_party/imgui/examples")

file(GLOB GUI_SRC "3rd_party/imgui/*.cpp"
  "3rd_party/imgui/examples/imgui_impl_glfw.cpp"
  "3rd_party/imgui/examples/imgui_impl_opengl3.cpp" )


add_definitions(-D_DEBUG)
add_definitions(-D_VERBOSE)
add_definitions(-D_PVD)
add_definitions(-DPVD_HOST="192.168.0.110")

add_subdirectory("3rd_party/optifuser")

file(GLOB_RECURSE PROJECT_SRC "src/*.h" "src/*.cpp")

add_library(mylib ${PROJECT_SRC})
target_link_libraries(mylib -Wl,--start-group
  libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a
  libPhysXCooking_static_64.a libPhysXExtensions_static_64.a
  libPhysXFoundation_static_64.a libPhysXPvdSDK_static_64.a libPhysX_static_64.a
  libPhysXVehicle_static_64.a libSnippetRender_static_64.a
  libSnippetUtils_static_64.a -Wl,--end-group pthread ${OPENGL_LIBRARY} glfw
  ${GLEW_LIBRARY} ${ASSIMP_LIBRARIES} dl optifuser stdc++fs tinyxml2)

add_executable(run "test/main.cpp")
target_link_libraries(run mylib)

