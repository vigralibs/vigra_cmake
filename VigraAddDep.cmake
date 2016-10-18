if(VigraAddDepIncluded)
    return()
endif()

include(CMakeParseArguments)
include(VAD_target_properties)

if(VAD_EXTERNAL_ROOT)
  message(STATUS "The root of external dependencies has been specified by the user: ${VAD_EXTERNAL_ROOT}")
else()
  set(VAD_EXTERNAL_ROOT ${PROJECT_SOURCE_DIR}/external)
  message(STATUS "The root of external dependencies has not been been specified by the user, setting it to the default: ${VAD_EXTERNAL_ROOT}")
endif()

function(git_clone NAME)
  find_package(Git REQUIRED)

  # Check if a git repo has been determined for the dependency.
  if(NOT VAD_${NAME}_GIT_REPO)
    message(FATAL_ERROR "A git clone was requested for dependency ${NAME}, but no repository variable has been set.")
  endif()

  # Don't do anything if the repo has been cloned already.
  if(EXISTS "${VAD_EXTERNAL_ROOT}/${NAME}")
    message(STATUS "The path '${VAD_EXTERNAL_ROOT}/${NAME}' already exists, skipping clone.")
    return()
  endif()

  message(STATUS "Git cloning repo '${VAD_${NAME}_GIT_REPO}' into '${VAD_EXTERNAL_ROOT}/${NAME}'.")

  # Build the command line options for the clone command.
  list(APPEND GIT_COMMAND_ARGS "clone" "${VAD_${NAME}_GIT_REPO}" "${NAME}")
  if(VAD_${NAME}_GIT_CLONE_OPTS)
    list(INSERT GIT_COMMAND_ARGS 1 ${VAD_${NAME}_GIT_CLONE_OPTS})
  endif()

  # Run the clone command.
  execute_process(COMMAND "${GIT_EXECUTABLE}" ${GIT_COMMAND_ARGS}
    WORKING_DIRECTORY "${VAD_EXTERNAL_ROOT}"
    RESULT_VARIABLE RES
    ERROR_VARIABLE OUT
    OUTPUT_VARIABLE OUT)
  if(RES)
    message(FATAL_ERROR "The clone command for '${NAME}' failed. The command arguments were: ${GIT_COMMAND_ARGS}\n\nThe output is:\n====\n${OUT}\n====")
  endif()

  message(STATUS "'${NAME}' was successfully cloned into '${VAD_EXTERNAL_ROOT}/${NAME}'")
endfunction()

# Override the builtin add_library() function so that the new library is appended to a global list
# if it is a non-global imported target. It will then be turned into a global imported target by find_package().
function(add_library NAME)
  list(FIND ARGN "IMPORTED" _IDX_IMP)
  list(FIND ARGN "GLOBAL" _IDX_GLOB)
  if(NOT _IDX_IMP EQUAL -1 AND _IDX_GLOB EQUAL -1)
    message(STATUS "Adding target '${NAME}' to the list of non-global imported targets.")
    get_property(_LIB_LIST GLOBAL PROPERTY _VAD_IMPORTED_NOGLOBAL_LIST)
    list(APPEND _LIB_LIST "${NAME}")
    set_property(GLOBAL PROPERTY _VAD_IMPORTED_NOGLOBAL_LIST ${_LIB_LIST})
  endif()
  _add_library(${NAME} ${ARGN})
  list(FIND ARGN "INTERFACE" _IDX_IFACE)
  list(FIND ARGN "ALIAS" _IDX_ALIAS)
  if(_IDX_IFACE EQUAL -1 AND _IDX_ALIAS EQUAL -1 AND NOT "${NAME}" MATCHES "^_VAD.*_STUB")
    file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/${NAME}.cmake_location" CONTENT "$<TARGET_FILE:${NAME}>")
  endif()
endfunction()

# An utility function to turn an imported target into a GLOBAL imported target.
# Imported targets have special visibility rules: they are not visible outside the subdir/subproject
# from which they were defined.
function(vad_make_imported_target_global NAME)
  # Check the target actually exists.
  if(NOT TARGET "${NAME}")
    message(FATAL_ERROR "'vad_make_imported_target_global()' was called with argument '${NAME}', but a target with that name does not exist.")
  endif()

  # Check if the target is imported. If it not, we just exit.
  get_target_property(IMP_PROP "${NAME}" IMPORTED)
  if(NOT IMP_PROP)
    message(FATAL_ERROR "Target '${NAME}' is not IMPORTED, no need to make it global.")
  endif()

  # The strategy here is as follows:
  # - we create a new imported target with global visibility, and we copy the properties of the original target;
  # - we create an INTERFACE target depending on the new target above;
  # - we create an alias for the INTERFACE target with the name of the original target.
  # It is necessary to go through the double indirection because of certain CMake rules regarding the allowed target
  # names.
  message(STATUS "Turning IMPORTED target '${NAME}' into a GLOBAL target.")
  get_target_property(TARGET_TYPE ${NAME} TYPE)
  message(STATUS "Target type is: ${TARGET_TYPE}")
  if(TARGET_TYPE MATCHES "INTERFACE_LIBRARY")
    add_library(_VAD_${NAME}_STUB INTERFACE IMPORTED GLOBAL)
  else()
    add_library(_VAD_${NAME}_STUB UNKNOWN IMPORTED GLOBAL)
  endif()
  foreach(TPROP ${VAD_TARGET_PROPERTIES})
      if(TARGET_TYPE MATCHES "INTERFACE_LIBRARY")
        list(FIND VAD_INTERFACE_LIBRARY_BLACKLISTED_PROPERTIES "${TPROP}" _IDX)
        if(NOT _IDX EQUAL -1)
          continue()
        endif()
      endif()
      get_target_property(PROP ${NAME} "${TPROP}")
      if(PROP)
          message(STATUS "Copying property '${TPROP}' of IMPORTED target '${NAME}': '${PROP}'")
          set_property(TARGET _VAD_${NAME}_STUB PROPERTY "${TPROP}" "${PROP}")
      endif()
  endforeach()
  # Create the final aliases. We need to remove any double colon from the original name as they are not allowed
  # in the names of targets in this context.
  string(REPLACE "::" "" NAME_NO_COLONS "${NAME}")
  add_library(_VAD_${NAME_NO_COLONS}_STUB_INTERFACE INTERFACE)
  target_link_libraries(_VAD_${NAME_NO_COLONS}_STUB_INTERFACE INTERFACE _VAD_${NAME}_STUB)
  add_library("${NAME}" ALIAS _VAD_${NAME_NO_COLONS}_STUB_INTERFACE)
endfunction()

# Override the builtin find_package() function to just call vigra_add_dep(). The purpose of this override is to make
# sure that any dep added via find_package() goes through the VAD machinery.
function(find_package NAME)
  vigra_add_dep(${NAME} SYSTEM ${ARGN})
endfunction()

# In addition to calling the builtin find_package(), this function
# will take care of exporting as global variables the variables defined by the builtin find_package(), and to make
# the imported targets defined by the builtin find_package() global.
function(find_package_plus NAME)
  message(STATUS "Invoking the patched 'find_package()' function for dependency ${NAME}.")
  # Get the list of the currently defined variables.
  get_cmake_property(_OLD_VARIABLES_${NAME} VARIABLES)
  # Call the original find_package().
  _find_package(${NAME} ${ARGN})
  # Detect new variables defined by find_package() and make them cached.
  get_cmake_property(_NEW_VARIABLES_${NAME} VARIABLES)
  # Remove duplicates in the new vars list.
  list(REMOVE_DUPLICATES _NEW_VARIABLES_${NAME})
  # Create a lower case version of the package name. We will use this in string matching below.
  string(TOLOWER "${NAME}" _NAME_LOW)
  # Detect the new variables by looping over the new vars list and comparing its elements to the old vars.
  foreach(_NEWVAR ${_NEW_VARIABLES_${NAME}})
      list(FIND _OLD_VARIABLES_${NAME} "${_NEWVAR}" _NEWVARIDX)
      if(_NEWVARIDX EQUAL -1)
          # New var was not found among the old ones. We check if it starts with
          # ${NAME} (case insensitively), in which case we will add it to the cached variables.
          string(TOLOWER "${_NEWVAR}" _NEWVAR_LOW)
          if(_NEWVAR_LOW MATCHES "^${_NAME_LOW}.*")
            # Make sure we don't store multiline strings in the cache, as that is not supported.
            string(REPLACE "\n" ";" _NEWVAR_NO_NEWLINES "${${_NEWVAR}}")
            message(STATUS "Storing new variable in cache: '${_NEWVAR}:${_NEWVAR_NO_NEWLINES}'")
            if(_NEWVAR_LOW MATCHES "librar" OR _NEWVAR_LOW MATCHES "include")
              # Variables which are likely to represent lib paths or include dirs are set as string variables,
              # so that they are visible from the GUI.
              set(${_NEWVAR} ${_NEWVAR_NO_NEWLINES} CACHE STRING "")
            else()
              # Otherwise, mark them as internal vars.
              set(${_NEWVAR} ${_NEWVAR_NO_NEWLINES} CACHE INTERNAL "")
            endif()
          endif()
      endif()
  endforeach()
  get_property(_IMP_NOGLOB_LIB_LIST GLOBAL PROPERTY _VAD_IMPORTED_NOGLOBAL_LIST)
  foreach(_NEWLIB ${_IMP_NOGLOB_LIB_LIST})
    vad_make_imported_target_global("${_NEWLIB}")
  endforeach()
  set_property(GLOBAL PROPERTY _VAD_IMPORTED_NOGLOBAL_LIST "")
endfunction()

# The default vad_system() function.
function(vad_system_default NAME)
  message(STATUS "Invoking the default implementation of vad_system() for dependency ${NAME}.")
  find_package_plus(${NAME} ${ARGN})
  # Check the FOUND flag.
  if(NOT ${NAME}_FOUND)
    set(VAD_${NAME}_SYSTEM_NOT_FOUND TRUE CACHE INTERNAL "")
  endif()
endfunction()

# A function to reset the hooks that are optionally defined in VAD files. Calling this function
# will reset the hooks to their default implementations.
function(vad_reset_hooks)
  function(vad_system)
    vad_system_default(${ARGN})
  endfunction()
  function(vad_live NAME)
    message(STATUS "Invoking the default implementation of vad_live() for dependency ${NAME}.")
    git_clone(${NAME})
    add_subdirectory("${VAD_EXTERNAL_ROOT}/${NAME}" "${VAD_EXTERNAL_ROOT}/${NAME}/build_external_dep")
  endfunction()
endfunction()

function(vigra_add_dep NAME)
  # Reset the hooks.
  vad_reset_hooks()

  # Parse the options.
  set(options SYSTEM LIVE)
  set(oneValueArgs GIT_REPO GIT_BRANCH GIT_COMMIT)
  set(multiValueArgs GIT_CLONE_OPTS)
  cmake_parse_arguments(ARG_VAD_${NAME} "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

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
      if(ARG_VAD_${NAME}_LIVE)
        message(STATUS "VAD file for dependency ${NAME} specifies git repo: ${GIT_REPO}")
      endif()
      set(VAD_${NAME}_GIT_REPO_FILE ${GIT_REPO})
      unset(GIT_REPO)
    endif()
    if(GIT_CLONE_OPTS)
      if(ARG_VAD_${NAME}_LIVE)
        message(STATUS "VAD file for dependency ${NAME} specifies git clone options: ${GIT_CLONE_OPTS}")
      endif()
      set(VAD_${NAME}_GIT_CLONE_OPTS_FILE ${GIT_CLONE_OPTS})
      unset(GIT_CLONE_OPTS)
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
    if(ARG_VAD_${NAME}_LIVE)
      message(STATUS "Final git repo address for dependency ${NAME} is from cache: ${VAD_${NAME}_GIT_REPO}")
    endif()
  else()
    if(ARG_VAD_${NAME}_GIT_REPO)
      # Git repo passed as function argument, overrides setting from file.
      if(ARG_VAD_${NAME}_LIVE)
        message(STATUS "Final git repo address for dependency ${NAME} is from function argument: ${ARG_VAD_${NAME}_GIT_REPO}")
      endif()
      set(VAD_${NAME}_GIT_REPO ${ARG_VAD_${NAME}_GIT_REPO} CACHE INTERNAL "")
    elseif(VAD_${NAME}_GIT_REPO_FILE)
      # Git repository coming from the file.
      if(ARG_VAD_${NAME}_LIVE)
        message(STATUS "Final git repo address for dependency ${NAME} is from file: ${VAD_${NAME}_GIT_REPO_FILE}")
      endif()
      set(VAD_${NAME}_GIT_REPO ${VAD_${NAME}_GIT_REPO_FILE} CACHE INTERNAL "")
    endif()
  endif()

  if(VAD_${NAME}_GIT_CLONE_OPTS)
    # Git clone options were passed from command line or this is not the first run.
    if(ARG_VAD_${NAME}_LIVE)
      message(STATUS "Final git clone options for dependency ${NAME} are from cache: ${VAD_${NAME}_GIT_CLONE_OPTS}")
    endif()
  else()
    if(ARG_VAD_${NAME}_GIT_CLONE_OPTS)
      # Git clone options passed as function argument, overrides setting from file.
      if(ARG_VAD_${NAME}_LIVE)
        message(STATUS "Final git clone options for dependency ${NAME} are from function argument: ${ARG_VAD_${NAME}_GIT_CLONE_OPTS}")
      endif()
      set(VAD_${NAME}_GIT_CLONE_OPTS ${ARG_VAD_${NAME}_GIT_CLONE_OPTS} CACHE INTERNAL "")
    elseif(VAD_${NAME}_GIT_CLONE_OPTS_FILE)
      # Git clone options coming from the file.
      if(ARG_VAD_${NAME}_LIVE)
        message(STATUS "Final git clone options for dependency ${NAME} are from file: ${VAD_${NAME}_GIT_CLONE_OPTS_FILE}")
      endif()
      set(VAD_${NAME}_GIT_CLONE_OPTS ${VAD_${NAME}_GIT_CLONE_OPTS_FILE} CACHE INTERNAL "")
    endif()
  endif()

  if(ARG_VAD_${NAME}_SYSTEM)
    vad_system(${NAME} ${ARG_VAD_${NAME}_UNPARSED_ARGUMENTS})
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
