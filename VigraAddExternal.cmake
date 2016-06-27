if(VigraAddExternalIncluded)
    return()
endif()

include(CMakeParseArguments)

OPTION(WITH_EXTERNAL_TESTS "Compile and run tests of external packages ?" OFF)

find_package(Git REQUIRED)

function(vigra_add_external AD_NAME)
  if(TARGET ${AD_NAME})
    message(STATUS "Dependency '${AD_NAME}' is already satisfied.")
    return()
  else()
    message(STATUS "Adding dependency '${AD_NAME}'.")
  endif()

  # Determine if the dependency has already been satisfied.
  # Create the external/ directory if it does not exist already.
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external")
    message(STATUS "Directory '${PROJECT_SOURCE_DIR}/external' does not exist, creating it.")
    file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/external")
  endif()

  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/${AD_NAME}")

    # Parse the options.
    set(options)
    set(oneValueArgs REPO BRANCH COMMIT)
    cmake_parse_arguments(AD "${options}" "${oneValueArgs}" "" ${ARGN})

    # Repo could come from cache variables, or from function option. Cached variable overrides the
    # function argument.
    if(DEPENDENCY_${AD_NAME}_REPO)
      message(STATUS "Cloning dependency '${AD_NAME}' from cached repo '${DEPENDENCY_${AD_NAME}_REPO}'.")
      set(AD_REPO "${DEPENDENCY_${AD_NAME}_REPO}")
    elseif(AD_REPO)
      message(STATUS "Cloning dependency '${AD_NAME}' from REPO '${AD_REPO}' specified in vigra_add_external().")
    else()
      message(FATAL_ERROR "No repository for dependency '${AD_NAME}' was specified in vigra_add_external() or the cache. Run cmake with '-DDEPENDENCY_${AD_NAME}_REPO=<repo_url>'.")
    endif()
    # Check the branch and commit arguments.
    if(AD_BRANCH AND AD_COMMIT)
      message(FATAL_ERROR "At most one of BRANCH and COMMIT can be passed to vigra_add_external().")
    endif()

    # Try to check out the dependency.
    execute_process(COMMAND "${GIT_EXECUTABLE}" "clone" "${AD_REPO}" "${AD_NAME}"
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/external"
      RESULT_VARIABLE RES
      ERROR_VARIABLE OUT
      OUTPUT_VARIABLE OUT)
    if(RES)
      message(FATAL_ERROR "The clone command for '${AD_NAME}' failed. The output is:\n====\n${OUT}\n====")
    endif()
    message(STATUS "'${AD_NAME}' was successfully cloned into '${PROJECT_SOURCE_DIR}/external/${AD_NAME}'")

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
  if(NOT WITH_EXTERNAL_TESTS)
    set(SKIP_TESTS 1)
  endif()
  add_subdirectory("${PROJECT_SOURCE_DIR}/external/${AD_NAME}")
  set(SKIP_TESTS 0)
endfunction()

# Mark as included.
set(VigraAddExternalIncluded YES)
