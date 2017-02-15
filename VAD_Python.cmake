include(CMakeParseArguments)
include(VigraAddDep)

function(vad_system)
  # Discover if the dep is required.
  set(options REQUIRED)
  cmake_parse_arguments(_FUNC_ARG "${options}" "" "" ${ARGN})

  # Start by looking for the Python interp. We will use it to infer all other info.
  vigra_add_dep(PythonInterp)
  if(NOT PythonInterp_FOUND)
    # If we don't find the Python interpreter, we cannot proceed.
    if(_FUNC_ARG_REQUIRED)
      # If the dep was required, then we will issue a fatal error.
      message(FATAL_ERROR "Could not find the Python interpreter, cannot proceed to locate the Python libraries.")
    endif()
    # Mark as unsatisfied and return.
    set(VAD_Python_SYSTEM_NOT_FOUND TRUE CACHE INTERNAL "")
    return()
  endif()

  # Prefix.
  if("${PYTHON_PREFIX}" STREQUAL "")
    message(STATUS "Determining Python prefix.")
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
                     "import sys; print(sys.exec_prefix)"
                      OUTPUT_VARIABLE PYTHON_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(PYTHON_PREFIX)
      message(STATUS "Python prefix detected: ${PYTHON_PREFIX}")
      set(PYTHON_PREFIX "${PYTHON_PREFIX}" CACHE PATH "Python prefix")
    else()
      message(FATAL_ERROR "Could not determine the Python prefix.")
    endif()
  else()
    message(STATUS "Python prefix from cache: ${PYTHON_PREFIX}")
  endif()

  # Includes.
  if("${PYTHON_INCLUDE_PATH}" STREQUAL "")
    message(STATUS "Determining Python include path.")
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
                    "from distutils.sysconfig import get_python_inc; print(get_python_inc())"
                  OUTPUT_VARIABLE PYTHON_INCLUDE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(PYTHON_INCLUDE_PATH)
      message(STATUS "Python include path detected: ${PYTHON_INCLUDE_PATH}")
      set(PYTHON_INCLUDE_PATH "${PYTHON_INCLUDE_PATH}" CACHE PATH "Python include path")
    else()
      message(FATAL_ERROR "Could not determine the Python include path.")
    endif()
  else()
    message(STATUS "Python include path from cache: ${PYTHON_INCLUDE_PATH}")
  endif()

  # Python library.
  if("${PYTHON_LIBRARIES}" STREQUAL "")
    message(STATUS "Determining Python libraries.")
    if(APPLE AND ${PYTHON_PREFIX} MATCHES ".*framework.*")
        set(PYTHON_LIBRARIES "${PYTHON_PREFIX}/Python"
            CACHE FILEPATH "Python libraries"
            FORCE)
    else()
        if(WIN32)
            set(PYTHON_LIBRARY_NAME python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
        else()
            execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
                             "from distutils.sysconfig import *; print(get_config_var('LDLIBRARY'))"
                              OUTPUT_VARIABLE PYTHON_LIBRARY_NAME OUTPUT_STRIP_TRAILING_WHITESPACE)
            execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
                             "from distutils.sysconfig import *; print(get_config_var('LIBDIR'))"
                               OUTPUT_VARIABLE PYTHON_LIBRARY_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
        endif()
        find_library(PYTHON_LIBRARIES ${PYTHON_LIBRARY_NAME} HINTS "${PYTHON_LIBRARY_PREFIX}" "${PYTHON_PREFIX}"
                     PATH_SUFFIXES lib lib64 libs DOC "Python libraries")
        unset(PYTHON_LIBRARY_PREFIX)
        if(PYTHON_LIBRARIES)
            message(STATUS "Python libraries detected: ${PYTHON_LIBRARIES}")
            set(PYTHON_LIBRARIES "${PYTHON_LIBRARIES}" CACHE PATH "Python libraries")
        else()
            message(FATAL_ERROR "Could not determine the Python libraries.")
        endif()
    endif()
  else()
    message(STATUS "Python libraries from cache: ${PYTHON_LIBRARIES}")
  endif()

  # Create the imported target.
  message(STATUS "Creating the Python::Python imported target.")
  add_library(Python::Python UNKNOWN IMPORTED)
  set_target_properties(Python::Python PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${PYTHON_INCLUDE_PATH}")
  set_property(TARGET Python::Python APPEND PROPERTY IMPORTED_LOCATION "${PYTHON_LIBRARIES}")
  make_imported_targets_global()
endfunction()
