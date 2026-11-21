set(patch_file "${CMAKE_CURRENT_LIST_DIR}/zlib-apple-fdopen.patch")
execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply "${patch_file}"
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
