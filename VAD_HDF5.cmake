set(GIT_REPO "https://github.com/live-clones/hdf5.git")

macro(h5_try_dir PATH)
  if (NOT _FOUND_H5CPP)
    find_path(_FOUND_H5CPP H5Cpp.h HINTS ${PATH} NO_DEFAULT_PATH NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_FIND_ROOT_PATH)
    if (_FOUND_H5CPP)
      set_target_properties(hdf5_cpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PATH})
    endif()
  endif()
endmacro()

function(vad_system)
  message("run VAD_SYSTEM for HDF5")
  
  # FIXME correct COMPONENTS handling...
  find_package_plus_no_import(${ARGN} COMPONENTS C CXX)
  
  if (NOT HDF5_FOUND)
    return()
  endif()
  
  if (NOT TARGET hdf5_cpp)
    add_library(hdf5_cpp INTERFACE IMPORTED)
    
    unset(_FOUND_H5CPP)
    unset(_FOUND_H5CPP CACHE)
    # search cxx header and add to hdf5_cpp target (workaround for various ways that findhdf5/hdf5config are broken on different systems)
    h5_try_dir("${HDF5_CXX_INCLUDE_DIRS}")
    h5_try_dir("${HDF5_CXX_INCLUDE_DIR}")
    h5_try_dir("${HDF5_C_INCLUDE_DIRS}")
    h5_try_dir("${HDF5_C_INCLUDE_DIR}")
    h5_try_dir("${HDF5_CXX_INCLUDE_DIRS}/cpp")
    h5_try_dir("${HDF5_CXX_INCLUDE_DIR}/cpp")
    h5_try_dir("${HDF5_C_INCLUDE_DIRS}/cpp")
    h5_try_dir("${HDF5_C_INCLUDE_DIR}/cpp")
    
    set_target_properties(hdf5_cpp PROPERTIES INTERFACE_LINK_LIBRARIES "${HDF5_CXX_LIBRARIES}")
  endif()
  make_imported_targets_global()
endfunction()

function(vad_live)
  message("run VAD_LIVE for HDF5")
  
  vigra_add_dep(Perl REQUIRED)
  
  if (NOT PERL_FOUND)
    message(FATAL_ERROR "perl required to build HDF5. Hint: try to set PERL_EXECUTABLE!")
  endif()
  
  git_clone(HDF5)
  
  set(HDF5_GENERATE_HEADERS ON CACHE BOOL "Rebuild Generated Files")
  set(HDF5_BUILD_EXAMPLES OFF CACHE BOOL "Build HDF5 Library Examples")
  # TODO uhh ohh nameing clash in inevitable...
  set(BUILD_TESTING OFF CACHE BOOL "Build HDF5 Unit Testing")
  
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/HDF5" "${CMAKE_BINARY_DIR}/external/HDF5")
  
  set(HDF5_INCLUDE_DIR "${VAD_EXTERNAL_ROOT}/HDF5/c++/src;${VAD_EXTERNAL_ROOT}/HDF5/src;${CMAKE_BINARY_DIR}/external/HDF5" CACHE STRING "" FORCE)
  
  set_property(TARGET hdf5_cpp-static APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/HDF5")
  
  add_library(hdf5_cpp ALIAS hdf5_cpp-static)
  # FIXME add hdf5 (C) target...
  
  set(HDF5_FOUND TRUE CACHE INTERNAL "" FORCE)
endfunction()
