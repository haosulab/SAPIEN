include(FetchContent)
FetchContent_Declare(
    openexr
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openexr.git
    GIT_TAG        v3.2.1
    GIT_SHALLOW TRUE
)


set(OPENEXR_INSTALL OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_TOOLS OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_WEBSITE OFF CACHE BOOL "" FORCE)
set(OPENEXR_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(OPENEXR_FORCE_INTERNAL_IMATH ON CACHE BOOL "" FORCE)

if(APPLE)
    set(OPENEXR_FORCE_INTERNAL_DEFLATE ON CACHE BOOL "Force using an internal libdeflate")
endif()
FetchContent_MakeAvailable(openexr)
