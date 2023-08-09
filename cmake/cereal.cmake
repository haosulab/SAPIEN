include(FetchContent)

set(SKIP_PERFORMANCE_COMPARISON 1)

FetchContent_Declare(
    cereal
    GIT_REPOSITORY https://github.com/USCiLab/cereal.git
    GIT_TAG        v1.3.2
)
FetchContent_MakeAvailableExclude(cereal)
