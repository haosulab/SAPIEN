if(TARGET pybind11)
  return()
endif()

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG 2943a27a14b507c67ca3e17c57bb74bbb7744f2b
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(pybind11)
