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
  message(STATUS "Git cloning repo ${REPO} into '${VAD_EXTERNAL_ROOT}/${NAME}'.")
  if(EXISTS "${VAD_EXTERNAL_ROOT}/${NAME}")
    message(STATUS "The path '${VAD_EXTERNAL_ROOT}/${NAME}' already exists, skipping clone.")
    return()
  endif()
  find_package(Git REQUIRED)
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
    add_subdirectory("${VAD_EXTERNAL_ROOT}/${NAME}" "${VAD_EXTERNAL_ROOT}/${NAME}/build_external_dep" EXCLUDE_FROM_ALL)
  endfunction()
endfunction()

function(vigra_add_dep VAD_NAME)
  # Reset the hooks.
  vad_reset_hooks()

  # Parse the options.
  set(options SYSTEM LIVE)
  set(oneValueArgs GIT_REPO_ARG GIT_BRANCH GIT_COMMIT)
  cmake_parse_arguments(VAD "${options}" "${oneValueArgs}" "" ${ARGN})

  # Validate options.
  # SYSTEM and LIVE cannot be present at the same time.
  if(VAD_SYSTEM AND VAD_LIVE)
    message(FATAL_ERROR "Dependency '${VAD_NAME}' was added both as a SYSTEM dependency and as a LIVE dependency: only one choice can be active.")
  endif()

  # Check if the dependency is already satisfied.
  if(VAD_DEP_${VAD_NAME}_SATISFIED)
    message(STATUS "Dependency '${VAD_NAME}' already satisfied, skipping.")
    return()
  endif()

  if(NOT VAD_SYSTEM AND NOT VAD_LIVE)
    # If neither SYSTEM nor LIVE were specified, assume SYSTEM.
    message(STATUS "Dependency '${VAD_NAME}' not specified as a SYSTEM or LIVE dependency, assuming SYSTEM.")
    set(VAD_SYSTEM YES)
  elseif(VAD_SYSTEM)
    # SYSTEM dependency required.
    message(STATUS "Dependency '${VAD_NAME}' specified as a SYSTEM dependency.")
  else()
    # LIVE dependency required.
    message(STATUS "Dependency '${VAD_NAME}' specified as a LIVE dependency.")
  endif()

  # if(VAD_GIT_BRANCH AND VAD_GIT_COMMIT)
  #   message(FATAL_ERROR "At most one of GIT_BRANCH and GIT_COMMIT can be used.")
  # endif()

  # First thing, we try to read the dep properties from the VAD file, if provided.
  find_file(VAD_${VAD_NAME}_FILE VAD_${VAD_NAME}.cmake ${CMAKE_MODULE_PATH})
  if(VAD_${VAD_NAME}_FILE)
    message(STATUS "VAD file 'VAD_${VAD_NAME}.cmake' was found at '${VAD_${VAD_NAME}_FILE}'.")
    include(${VAD_${VAD_NAME}_FILE})
    if(GIT_REPO)
      message(STATUS "Git repo for dependency ${VAD_NAME} read from VAD file: ${GIT_REPO}")
      set(VAD_${NAME}_GIT_REPO_FILE ${GIT_REPO})
      unset(GIT_REPO)
    endif()
    if(GIT_BRANCH)
      message(STATUS "Git branch for dependency ${VAD_NAME} read from VAD file: ${GIT_BRANCH}")
      set(VAD_GIT_BRANCH_CUR ${GIT_BRANCH})
      unset(GIT_BRANCH)
    endif()
    if(GIT_COMMIT)
      message(STATUS "Git commit for dependency ${VAD_NAME} read from VAD file: ${GIT_COMMIT}")
      set(VAD_GIT_COMMIT_CUR ${GIT_COMMIT})
      unset(GIT_COMMIT)
    endif()
    unset(VAD_${VAD_NAME}_FILE CACHE)
  else()
    message(STATUS "No VAD file 'VAD_${VAD_NAME}.cmake' was found.")
  endif()

  # TODO: same for branch and commit.
  if(VAD_${VAD_NAME}_GIT_REPO)
    # Git repo was passed from command line or this is not the first run.
    message(STATUS "Final git repo address for dependency ${VAD_NAME} is from cache: ${VAD_${VAD_NAME}_GIT_REPO}")
  else()
    if(GIT_REPO_ARG)
      # Git repo passed as function argument, overrides setting from file.
      message(STATUS "Final git repo address for dependency ${VAD_NAME} is from function argument: ${GIT_REPO_ARG}")
      set(VAD_${VAD_NAME}_GIT_REPO ${GIT_REPO_ARG} CACHE INTERNAL "")
    elseif(VAD_${NAME}_GIT_REPO_FILE)
      # Git repository coming from the file.
      message(STATUS "Final git repo address for dependency ${VAD_NAME} is from file: ${VAD_${NAME}_GIT_REPO_FILE}")
      set(VAD_${VAD_NAME}_GIT_REPO ${VAD_${NAME}_GIT_REPO_FILE} CACHE INTERNAL "")
    endif()
  endif()

  if(VAD_SYSTEM)
    vad_system(${VAD_NAME})
  elseif(VAD_LIVE)
    # Create the external deps directory if it does not exist already.
    if(NOT EXISTS "${VAD_EXTERNAL_ROOT}")
      message(STATUS "Directory '${VAD_EXTERNAL_ROOT}' does not exist, creating it.")
      file(MAKE_DIRECTORY "${VAD_EXTERNAL_ROOT}")
    endif()
    vad_live(${VAD_NAME})
  endif()

  # Mark the dep as satisfied.
  message(STATUS "Marking dependency ${VAD_NAME} as satisfied.")
  # NOTE: the flag must escape the function scope.
  set(VAD_DEP_${VAD_NAME}_SATISFIED YES PARENT_SCOPE)
endfunction()

# Mark as included.
set(VigraAddDepIncluded YES)
