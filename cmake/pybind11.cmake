if(TARGET pybind11)
  return()
endif()

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG b6444460eeddc2965ab1a49c6c50c83073779489
  # GIT_TAG v2.10.4
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(pybind11)
