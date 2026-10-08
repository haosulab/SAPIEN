include(FetchContent)
FetchContent_Declare(
    oidn
    URL https://github.com/OpenImageDenoise/oidn/releases/download/v2.0.1/oidn-2.0.1.src.tar.gz
    URL_HASH MD5=9e13ff3d9eb640e923b699bea1c8d419
)

set(OIDN_APPS OFF CACHE BOOL "" FORCE)
set(OIDN_DEVICE_CPU OFF CACHE BOOL "" FORCE)
set(OIDN_DEVICE_CUDA ON CACHE BOOL "" FORCE)
set(OIDN_DEVICE_HIP OFF CACHE BOOL "" FORCE)
set(OIDN_DEVICE_SYCL OFF CACHE BOOL "" FORCE)
set(OIDN_FILTER_RT ON CACHE BOOL "" FORCE)
set(OIDN_FILTER_RTLIGHTMAP OFF CACHE BOOL "" FORCE)
set(OIDN_INSTALL_DEPENDENCIES OFF CACHE BOOL "" FORCE)

FetchContent_GetProperties(oidn)
if(NOT oidn_POPULATED)
  FetchContent_Populate(oidn)
  add_subdirectory(${oidn_SOURCE_DIR} ${oidn_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# include(FetchContent)
# FetchContent_Declare(
#   oidn
#   URL https://github.com/OpenImageDenoise/oidn/releases/download/v2.0.1/oidn-2.0.1.x86_64.linux.tar.gz
#   URL_HASH MD5=c021d2fa5b41878dbc62d4e1f8469e67
# )
# FetchContent_MakeAvailable(oidn)


# add_library(OpenImageDenoise SHARED IMPORTED)
# set_target_properties(OpenImageDenoise PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${oidn_SOURCE_DIR}/include")
# set_target_properties(OpenImageDenoise PROPERTIES IMPORTED_LOCATION
#   ${oidn_SOURCE_DIR}/lib/libOpenImageDenoise.so.2.0.1
#   ${oidn_SOURCE_DIR}/lib/libOpenImageDenoise_core.so.2.0.1
#   ${oidn_SOURCE_DIR}/lib/libOpenImageDenoise_device_cuda.so.2.0.1
# )
