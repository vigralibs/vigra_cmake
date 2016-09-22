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

function(vigra_add_dep VAD_NAME)
  # Parse the options.
  set(options SYSTEM LIVE)
  set(oneValueArgs GIT_REPO GIT_BRANCH GIT_COMMIT)
  cmake_parse_arguments(VAD "${options}" "${oneValueArgs}" "" ${ARGN})

  # Validate options.
  # SYSTEM and LIVE cannot be present at the same time.
  if(VAD_SYSTEM AND VAD_LIVE)
    message(FATAL_ERROR "Dependency '${VAD_NAME}' was added both as a SYSTEM dependency and as a LIVE dependency: only one choice can be active.")
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

  # First thing, we try to read the dep properties from the cmake file, if provided.
  find_file(VAD_${VAD_NAME}_FILE VAD_${VAD_NAME}.cmake ${CMAKE_MODULE_PATH})
  if(VAD_${VAD_NAME}_FILE)
    message(STATUS "VAD file 'VAD_${VAD_NAME}.cmake' was found at '${VAD_${VAD_NAME}_FILE}'.")
    include(${VAD_${VAD_NAME}_FILE})
    if(GIT_REPO)
      message(STATUS "Git repo for dependency ${VAD_NAME} read from VAD file: ${GIT_REPO}")
      set(VAD_GIT_REPO_CUR ${GIT_REPO})
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
  else()
    message(STATUS "No VAD file 'VAD_${VAD_NAME}.cmake' was found.")
  endif()

  # Second, we try to see if options were passed in as function arguments. These will override the values in the files,
  # if any.
  if(VAD_GIT_REPO)
    if(VAD_GIT_REPO_CUR)
      message(STATUS "Overriding file-provided git repo for dependency ${VAD_NAME} with value from from function argument: ${VAD_GIT_REPO}")
    else()
      message(STATUS "Git repo for dependency ${VAD_NAME} from function argument: ${VAD_GIT_REPO}")
    endif()
    set(VAD_GIT_REPO_CUR ${VAD_GIT_REPO})
  endif()

  # TODO branch, commit.
endfunction()

# Mark as included.
set(VigraAddDepIncluded YES)
