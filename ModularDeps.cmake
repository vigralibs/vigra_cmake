if(ModularDepsIncluded)
    return()
endif()

find_package(Git REQUIRED)

function(add_dependency DEP_NAME DEP_REPO)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps")
    message(STATUS "The '${CMAKE_CURRENT_SOURCE_DIR}/deps' directory already exists.")
  else()
    message(STATUS "The '${CMAKE_CURRENT_SOURCE_DIR}/deps' directory does not exist, creating it.")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/deps")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/${DEP_NAME}")
    message(STATUS "The '${DEP_NAME}' dir exists already.")
  else()
    message(STATUS "Cloning the '${DEP_NAME}' dependency.")
    execute_process(COMMAND "${GIT_EXECUTABLE}" "clone" "${DEP_REPO}" "${DEP_NAME}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/deps"
      RESULT_VARIABLE RES
      ERROR_VARIABLE OUT
      OUTPUT_VARIABLE OUT)
    if(RES)
      message(FATAL_ERROR "The clone command for '${DEP_NAME}' failed. The output is:\n====\n${OUT}\n====")
    endif()
    message(STATUS "'${DEP_NAME}' was successfully cloned.")
  endif()
endfunction()

# Mark as included.
set(ModularDepsIncluded YES)
