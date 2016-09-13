if(VigraAddExternalIncluded)
    return()
endif()

include(CMakeParseArguments)

OPTION(WITH_EXTERNAL_TESTS "Compile and run tests of external packages ?" OFF)

if(DEPS_EXTERNAL_ROOT)
  message(STATUS "The root of external dependencies has been specified by the user: '${DEPS_EXTERNAL_ROOT}'")
else()
  set(DEPS_EXTERNAL_ROOT ${PROJECT_SOURCE_DIR}/external)
  message(STATUS "The root of external dependencies has not been been specified by the user, setting it to the default: '${DEPS_EXTERNAL_ROOT}'")
endif()

find_package(Git REQUIRED)

function(vigra_add_external AD_NAME)
  # Determine if the dependency has already been satisfied.
  if(TARGET ${AD_NAME})
    message(STATUS "Dependency '${AD_NAME}' is already satisfied.")
    return()
  else()
    message(STATUS "Adding dependency '${AD_NAME}'.")
  endif()

  # Create the external deps directory if it does not exist already.
  if(NOT EXISTS "${DEPS_EXTERNAL_ROOT}")
    message(STATUS "Directory '${DEPS_EXTERNAL_ROOT}' does not exist, creating it.")
    file(MAKE_DIRECTORY "${DEPS_EXTERNAL_ROOT}")
  endif()

  # Parse the options.
  set(options SYSTEM)
  set(oneValueArgs REPO BRANCH COMMIT)
  cmake_parse_arguments(AD "${options}" "${oneValueArgs}" "" ${ARGN})

  if(AD_SYSTEM OR NOT AD_REPO)
    if(NOT AD_REPO)
      message(STATUS "Dependency '${AD_NAME}' was not provided with a repository address, assuming system-wide dependency.")
    else()
      message(STATUS "Dependency '${AD_NAME}' declared as a system-wide dependency.")
    endif()
    find_file(AD_SYSTEM_FILE ${AD_NAME}_system.cmake ${CMAKE_MODULE_PATH})
    if(AD_SYSTEM_FILE)
      message(STATUS "Found system file for ${AD_NAME} at ${AD_SYSTEM_FILE}.")
      include(${AD_SYSTEM_FILE})
    else()
      message(STATUS "No system file for ${AD_NAME} at was found, locating package via 'find_package()'.")
      find_package(${AD_NAME} REQUIRED)
    endif()
    unset(AD_SYSTEM_FILE CACHE)
  else()
    # Test if we need to do the initial repo clone.
    if(NOT EXISTS "${DEPS_EXTERNAL_ROOT}/${AD_NAME}")
      # Repo could come from cache variables, or from function option. Cached variable overrides the
      # function argument.
      if(DEPENDENCY_${AD_NAME}_REPO)
        message(STATUS "Cloning dependency '${AD_NAME}' from cached repo '${DEPENDENCY_${AD_NAME}_REPO}'.")
        set(AD_REPO "${DEPENDENCY_${AD_NAME}_REPO}")
      else()
        message(STATUS "Cloning dependency '${AD_NAME}' from REPO '${AD_REPO}' specified in vigra_add_external().")
      endif()
      # Check the branch and commit arguments.
      if(AD_BRANCH AND AD_COMMIT)
        message(FATAL_ERROR "At most one of BRANCH and COMMIT can be passed to vigra_add_external().")
      endif()

      # Try to check out the dependency.
      execute_process(COMMAND "${GIT_EXECUTABLE}" "clone" "${AD_REPO}" "${AD_NAME}"
        WORKING_DIRECTORY "${DEPS_EXTERNAL_ROOT}"
        RESULT_VARIABLE RES
        ERROR_VARIABLE OUT
        OUTPUT_VARIABLE OUT)
      if(RES)
        message(FATAL_ERROR "The clone command for '${AD_NAME}' failed. The output is:\n====\n${OUT}\n====")
      endif()
      message(STATUS "'${AD_NAME}' was successfully cloned into '${DEPS_EXTERNAL_ROOT}/${AD_NAME}'")

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
    # The repo has been checked out, proceed to read the targets.
    if(NOT WITH_EXTERNAL_TESTS)
      set(SKIP_TESTS 1)
    endif()
    find_file(AD_REPO_FILE ${AD_NAME}_repo.cmake ${CMAKE_MODULE_PATH})
    if(AD_REPO_FILE)
      message(STATUS "Found repo file for ${AD_NAME} at ${AD_REPO_FILE}.")
      include(${AD_REPO_FILE})
    else()
      message(STATUS "No repo file for ${AD_NAME} was found.")
      add_subdirectory("${DEPS_EXTERNAL_ROOT}/${AD_NAME}" "${DEPS_EXTERNAL_ROOT}/${AD_NAME}/build_external_dep")
    endif()
    unset(AD_REPO_FILE CACHE)
    set(SKIP_TESTS 0)
  endif()
endfunction()

# Mark as included.
set(VigraAddExternalIncluded YES)
