if(TARGET easy_profiler)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    easy_profiler
    GIT_REPOSITORY https://github.com/yse/easy_profiler.git
    GIT_TAG v2.1.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

FetchContent_MakeAvailableExclude(easy_profiler)
set_target_properties(easy_profiler PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
