if(ModularDepsIncluded)
    return()
endif()

find_package(Git REQUIRED)

function(add_dependency)
  set(options)
  set(oneValueArgs NAME REPO)
  cmake_parse_arguments(AD "${options}" "${oneValueArgs}" "" ${ARGN})
  set(DEP_NAME "${AD_NAME}")
  set(DEP_REPO "${AD_REPO}")

  # Determine if the dependency has already been satisfied.
  if(${DEP_NAME}_DEP_SATISFIED)
    message(STATUS "The dependency '${DEP_NAME}' is already satisfied.")
    return()
  endif()
  message(STATUS "Adding the dependency '${DEP_NAME}'.")

  # Create the deps/ directory if it does not exist already.
  if(EXISTS "${PROJECT_SOURCE_DIR}/deps")
    message(STATUS "The '${PROJECT_SOURCE_DIR}/deps' directory already exists.")
  else()
    message(STATUS "The '${PROJECT_SOURCE_DIR}/deps' directory does not exist, creating it.")
    file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/deps")
  endif()

  # Try to check out the dependency.
  if(EXISTS "${PROJECT_SOURCE_DIR}/deps/${DEP_NAME}")
    message(STATUS "The '${DEP_NAME}' dir exists already.")
  else()
    message(STATUS "Cloning the '${DEP_NAME}' dependency.")
    execute_process(COMMAND "${GIT_EXECUTABLE}" "clone" "${DEP_REPO}" "${DEP_NAME}"
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/deps"
      RESULT_VARIABLE RES
      ERROR_VARIABLE OUT
      OUTPUT_VARIABLE OUT)
    if(RES)
      message(FATAL_ERROR "The clone command for '${DEP_NAME}' failed. The output is:\n====\n${OUT}\n====")
    endif()
    message(STATUS "'${DEP_NAME}' was successfully cloned.")
  endif()

  # Add the subdirectory of the dependency.
  add_subdirectory("${PROJECT_SOURCE_DIR}/deps/${DEP_NAME}")

  # Mark the dependency as satisfied.
  set(${DEP_NAME}_DEP_SATISFIED YES CACHE BOOL "${DEP_NAME} dependency.")
  mark_as_advanced(${DEP_NAME}_DEP_SATISFIED)
endfunction()

# Mark as included.
set(ModularDepsIncluded YES)
