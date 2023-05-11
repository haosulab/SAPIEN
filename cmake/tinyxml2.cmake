if(TARGET tinyxml2)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG        9.0.0
)


FetchContent_MakeAvailableExclude(tinyxml2)
set_target_properties(tinyxml2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
