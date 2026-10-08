include(FetchContent)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        0.9.9.8
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(glm)

get_target_property(_inc glm INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(glm PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
target_include_directories(glm SYSTEM INTERFACE
    $<BUILD_INTERFACE:${_inc}>
    $<INSTALL_INTERFACE:include>
)
