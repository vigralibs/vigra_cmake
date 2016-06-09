if(ModularDepsIncluded)
    return()
endif()

find_package(Git REQUIRED)

# The global dependency list.
set(DEPENDENCY_LIST)

function(add_dependency DEP_NAME DEP_REPO)
  # Determine if the dependency has already been added.
  set(DEP_INDEX)
  list(FIND DEPENDENCY_LIST "${DEP_NAME}" DEP_INDEX)
  if(DEP_INDEX GREATER -1)
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

  # Add the dependency to the global dependency list.
  # NOTE: we cannot use list(APPEND) here because DEPENDENCY_LIST is a global variable,
  # and in order to access it we need to use the PARENT_SCOPE option of the set() command.
  set(DEPENDENCY_LIST ${DEPENDENCY_LIST} "${DEP_NAME}" PARENT_SCOPE)

  # Add the subdirectory of the dependency.
  add_subdirectory("${PROJECT_SOURCE_DIR}/deps/${DEP_NAME}")
endfunction()

# Mark as included.
set(ModularDepsIncluded YES)
