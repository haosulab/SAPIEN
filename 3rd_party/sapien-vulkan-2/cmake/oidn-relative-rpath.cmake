set(patch_file "${CMAKE_CURRENT_LIST_DIR}/oidn-relative-rpath.patch")
if(NOT EXISTS "${SOURCE_DIR}/CMakeLists.txt" OR NOT EXISTS "${SOURCE_DIR}/devices/CMakeLists.txt")
    message(FATAL_ERROR "OIDN patch targets are missing in ${SOURCE_DIR}")
endif()
get_filename_component(source_parent_dir "${SOURCE_DIR}" DIRECTORY)
set(ENV{GIT_CEILING_DIRECTORIES} "${source_parent_dir}")
execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply --verbose "${patch_file}"
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE application_exit
    OUTPUT_VARIABLE application_output
    ERROR_VARIABLE application_error
)
if(NOT application_exit EQUAL 0)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${patch_file}"
        WORKING_DIRECTORY "${SOURCE_DIR}"
        RESULT_VARIABLE already_patched_exit
        OUTPUT_QUIET
        ERROR_QUIET
    )
    if(NOT already_patched_exit EQUAL 0)
        message(FATAL_ERROR "${application_output}\n${application_error}")
    endif()
endif()
if("${application_output}\n${application_error}" MATCHES "[Ss]kipped patch")
    message(FATAL_ERROR "${application_output}\n${application_error}")
endif()
file(SHA256 "${SOURCE_DIR}/CMakeLists.txt" root_cmake_sha256)
file(SHA256 "${SOURCE_DIR}/devices/CMakeLists.txt" devices_cmake_sha256)
if(NOT root_cmake_sha256 STREQUAL "bc066b8e572bfdf96d9411e238ca1d75892856e22b4a4e9ccabbe78dbb07ed78" OR
   NOT devices_cmake_sha256 STREQUAL "9e27285377844c824adc2df3cffea1067ede63f778ecb599f19db38467bd50ac")
    message(FATAL_ERROR "OIDN patch did not produce the expected patched sources in ${SOURCE_DIR}\n${application_output}\n${application_error}")
endif()
message(STATUS "OIDN patch application verified in ${SOURCE_DIR}")
