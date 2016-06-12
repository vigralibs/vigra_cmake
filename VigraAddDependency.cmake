if(VigraAddDependencyIncluded)
    return()
endif()

find_package(Git REQUIRED)

function(add_dependency AD_NAME)
  if(TARGET ${AD_NAME})
    message(STATUS "Dependency '${AD_NAME}' is already satisfied.")
    return()
  else()
    message(STATUS "Adding dependency '${AD_NAME}'.")
  endif()

  # Determine if the dependency has already been satisfied.
  # Create the deps/ directory if it does not exist already.
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/deps")
    message(STATUS "Directory '${PROJECT_SOURCE_DIR}/deps' does not exist, creating it.")
    file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/deps")
  endif()

  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/deps/${AD_NAME}")

    # Parse the options.
    set(options)
    set(oneValueArgs REPO BRANCH COMMIT)
    cmake_parse_arguments(AD "${options}" "${oneValueArgs}" "" ${ARGN})

    # Repo could come from cache variables, or from function option. Cached variable overrides the
    # function argument.
    if(DEPENDENCY_${AD_NAME}_REPO)
      message(STATUS "Setting the repository for dependency '${AD_NAME}' to the value from cache '${DEPENDENCY_${AD_NAME}_REPO}'.")
      set(AD_REPO "${DEPENDENCY_${AD_NAME}_REPO}")
    elseif(AD_REPO)
      message(STATUS "Setting the repository for dependency '${AD_NAME}' to the value '${AD_REPO}' specified to the add_dependency() call.")
    else()
      message(FATAL_ERROR "The repository of the dependency was not specified in the add_dependency() call, nor it was found in the cached variables.")
    endif()
    # Check the branch and commit arguments.
    if(AD_BRANCH AND AD_COMMIT)
      message(FATAL_ERROR "At most one of branch and commit must be specified when using add_dependency().")
    endif()

    # Try to check out the dependency.
    message(STATUS "Cloning the '${AD_NAME}' dependency.")
    execute_process(COMMAND "${GIT_EXECUTABLE}" "clone" "${AD_REPO}" "${AD_NAME}"
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/deps"
      RESULT_VARIABLE RES
      ERROR_VARIABLE OUT
      OUTPUT_VARIABLE OUT)
    if(RES)
      message(FATAL_ERROR "The clone command for '${AD_NAME}' failed. The output is:\n====\n${OUT}\n====")
    endif()
    message(STATUS "'${AD_NAME}' was successfully cloned. The git clone is located at '${PROJECT_SOURCE_DIR}/deps/${AD_NAME}'")

    if(AD_REPO)
      set(DEPENDENCY_${AD_NAME}_REPO "${AD_REPO}" CACHE STRING "")
      mark_as_advanced(DEPENDENCY_${AD_NAME}_REPO)
    endif()
    if(AD_BRANCH)
      set(DEPENDENCY_${AD_NAME}_BRANCH "${AD_BRANCH}" CACHE STRING "")
      mark_as_advanced(DEPENDENCY_${AD_NAME}_BRANCH)
    endif()
    if(AD_COMMIT)
      set(DEPENDENCY_${AD_NAME}_COMMIT "${AD_COMMIT}" CACHE STRING "")
      mark_as_advanced(DEPENDENCY_${AD_NAME}_COMMIT)
    endif()
  endif()

  # Add the subdirectory of the dependency.
  add_subdirectory("${PROJECT_SOURCE_DIR}/deps/${AD_NAME}")
endfunction()

# Mark as included.
set(VigraAddDependencyIncluded YES)
