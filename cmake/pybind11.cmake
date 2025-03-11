if(TARGET pybind11)
  return()
endif()

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG 06e8ee2e357fc2fd6e36de431fa0ca0049aafc7d
)

FetchContent_MakeAvailable(pybind11)
