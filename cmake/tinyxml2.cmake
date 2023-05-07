if(TARGET tinyxml2)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG        9.0.0
)

FetchContent_GetProperties(tinyxml)
if(NOT tinyxml2_POPULATED)
  FetchContent_Populate(tinyxml2)
  add_subdirectory(${tinyxml2_SOURCE_DIR} ${tinyxml2_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

set_target_properties(tinyxml2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
