if(VigraAddDepIncluded)
    return()
endif()

include(CMakeParseArguments)

if(VAD_EXTERNAL_ROOT)
  message(STATUS "The root of external dependencies has been specified by the user: ${VAD_EXTERNAL_ROOT}")
else()
  set(VAD_EXTERNAL_ROOT ${PROJECT_SOURCE_DIR}/external)
  message(STATUS "The root of external dependencies has not been been specified by the user, setting it to the default: ${VAD_EXTERNAL_ROOT}")
endif()

function(git_clone REPO NAME)
  find_package(Git REQUIRED)
  message(STATUS "Git cloning repo ${REPO} into '${VAD_EXTERNAL_ROOT}/${NAME}'.")
  if(EXISTS "${VAD_EXTERNAL_ROOT}/${NAME}")
    message(STATUS "The path '${VAD_EXTERNAL_ROOT}/${NAME}' already exists, skipping clone.")
    return()
  endif()
  # Try to check out the dependency.
  execute_process(COMMAND "${GIT_EXECUTABLE}" "clone" "${REPO}" "${NAME}"
    WORKING_DIRECTORY "${VAD_EXTERNAL_ROOT}"
    RESULT_VARIABLE RES
    ERROR_VARIABLE OUT
    OUTPUT_VARIABLE OUT)
  if(RES)
    message(FATAL_ERROR "The clone command for '${NAME}' failed. The output is:\n====\n${OUT}\n====")
  endif()
  message(STATUS "'${NAME}' was successfully cloned into '${VAD_EXTERNAL_ROOT}/${NAME}'")
endfunction()

# A function to reset the hooks that are optionally defined in VAD files. Calling this function
# will reset the hooks to their default implementations.
function(vad_reset_hooks)
  function(vad_system NAME)
    message(STATUS "Invoking the default implementation of vad_system() for dependency ${NAME}.")
    find_package(${NAME})
  endfunction()
  function(vad_live NAME)
    message(STATUS "Invoking the default implementation of vad_live() for dependency ${NAME}.")
    if(NOT VAD_${NAME}_GIT_REPO)
      message(FATAL_ERROR "A git clone was requested for dependency ${NAME}, but no repository variable has been set.")
    endif()
    git_clone(${VAD_${NAME}_GIT_REPO} ${NAME})
    add_subdirectory("${VAD_EXTERNAL_ROOT}/${NAME}" "${VAD_EXTERNAL_ROOT}/${NAME}/build_external_dep")
  endfunction()
endfunction()

function(vigra_add_dep NAME)
  # Reset the hooks.
  vad_reset_hooks()

  # Parse the options.
  set(options SYSTEM LIVE)
  set(oneValueArgs GIT_REPO GIT_BRANCH GIT_COMMIT)
  cmake_parse_arguments(ARG_VAD_${NAME} "${options}" "${oneValueArgs}" "" ${ARGN})

  # Validate options.
  # SYSTEM and LIVE cannot be present at the same time.
  if(ARG_VAD_${NAME}_SYSTEM AND ARG_VAD_${NAME}_LIVE)
    message(FATAL_ERROR "Dependency '${NAME}' was added both as a SYSTEM dependency and as a LIVE dependency: only one choice can be active.")
  endif()

  # Check if the dependency is already satisfied.
  get_property(_VAD_DEP_${NAME}_SATISFIED GLOBAL PROPERTY VAD_DEP_${NAME}_SATISFIED)
  if(_VAD_DEP_${NAME}_SATISFIED)
    message(STATUS "Dependency '${NAME}' already satisfied, skipping.")
    return()
  endif()

  if(NOT ARG_VAD_${NAME}_SYSTEM AND NOT ARG_VAD_${NAME}_LIVE)
    # If neither SYSTEM nor LIVE were specified, assume SYSTEM.
    message(STATUS "Dependency '${NAME}' not specified as a SYSTEM or LIVE dependency, assuming SYSTEM.")
    set(ARG_VAD_${NAME}_SYSTEM YES)
  elseif(ARG_VAD_${NAME}_SYSTEM)
    # SYSTEM dependency required.
    message(STATUS "Dependency '${NAME}' specified as a SYSTEM dependency.")
  else()
    # LIVE dependency required.
    message(STATUS "Dependency '${NAME}' specified as a LIVE dependency.")
  endif()

  # if(VAD_GIT_BRANCH AND VAD_GIT_COMMIT)
  #   message(FATAL_ERROR "At most one of GIT_BRANCH and GIT_COMMIT can be used.")
  # endif()

  # First thing, we try to read the dep properties from the VAD file, if provided.
  find_file(VAD_${NAME}_FILE VAD_${NAME}.cmake ${CMAKE_MODULE_PATH})
  if(VAD_${NAME}_FILE)
    message(STATUS "VAD file 'VAD_${NAME}.cmake' was found at '${VAD_${NAME}_FILE}'. The VAD file will now be parsed.")
    include(${VAD_${NAME}_FILE})
    if(GIT_REPO)
      message(STATUS "VAD file for dependency ${NAME} specifies git repo: ${GIT_REPO}")
      set(VAD_${NAME}_GIT_REPO_FILE ${GIT_REPO})
      unset(GIT_REPO)
    endif()
    # TODO: branch/commit.
    if(GIT_BRANCH)
      message(STATUS "Git branch for dependency ${NAME} read from VAD file: ${GIT_BRANCH}")
      set(VAD_GIT_BRANCH_CUR ${GIT_BRANCH})
      unset(GIT_BRANCH)
    endif()
    if(GIT_COMMIT)
      message(STATUS "Git commit for dependency ${NAME} read from VAD file: ${GIT_COMMIT}")
      set(VAD_GIT_COMMIT_CUR ${GIT_COMMIT})
      unset(GIT_COMMIT)
    endif()
  else()
    message(STATUS "No VAD file 'VAD_${NAME}.cmake' was found.")
  endif()
  # Unset the variable from cache.
  unset(VAD_${NAME}_FILE CACHE)

  # TODO: same for branch and commit.
  if(VAD_${NAME}_GIT_REPO)
    # Git repo was passed from command line or this is not the first run.
    message(STATUS "Final git repo address for dependency ${NAME} is from cache: ${VAD_${NAME}_GIT_REPO}")
  else()
    if(ARG_VAD_${NAME}_GIT_REPO)
      # Git repo passed as function argument, overrides setting from file.
      message(STATUS "Final git repo address for dependency ${NAME} is from function argument: ${ARG_VAD_${NAME}_GIT_REPO}")
      set(VAD_${NAME}_GIT_REPO ${ARG_VAD_${NAME}_GIT_REPO} CACHE INTERNAL "")
    elseif(VAD_${NAME}_GIT_REPO_FILE)
      # Git repository coming from the file.
      message(STATUS "Final git repo address for dependency ${NAME} is from file: ${VAD_${NAME}_GIT_REPO_FILE}")
      set(VAD_${NAME}_GIT_REPO ${VAD_${NAME}_GIT_REPO_FILE} CACHE INTERNAL "")
    endif()
  endif()

  if(ARG_VAD_${NAME}_SYSTEM)
    vad_system(${NAME})
    if(VAD_${NAME}_SYSTEM_NOT_FOUND)
      message(STATUS "Dependency ${NAME} was not found system-wide, vigra_add_dep() will exit without marking the dependency as satisfied.")
      unset(VAD_${NAME}_SYSTEM_NOT_FOUND CACHE)
      vad_reset_hooks()
      return()
    endif()
  elseif(ARG_VAD_${NAME}_LIVE)
    # Create the external deps directory if it does not exist already.
    if(NOT EXISTS "${VAD_EXTERNAL_ROOT}")
      message(STATUS "Directory '${VAD_EXTERNAL_ROOT}' does not exist, creating it.")
      file(MAKE_DIRECTORY "${VAD_EXTERNAL_ROOT}")
    endif()
    vad_live(${NAME})
  endif()

  # Mark the dep as satisfied.
  message(STATUS "Marking dependency ${NAME} as satisfied.")
  # NOTE: the flag be set as a "global" variable, that is available from everywhere, but not in the cache - otherwise
  # subsequent cmake runs will break. Emulate global variables via global properties:
  # http://stackoverflow.com/questions/19345930/cmake-lost-in-the-concept-of-global-variables-and-parent-scope-or-add-subdirec
  set_property(GLOBAL PROPERTY VAD_DEP_${NAME}_SATISFIED YES)

  # Reset the hooks at the end as well for cleanup purposes.
  vad_reset_hooks()
endfunction()

# Mark as included.
set(VigraAddDepIncluded YES)
