set(GIT_REPO "https://github.com/live-clones/hdf5.git")

function(vad_system)
  message("run VAD_SYSTEM for HDF5")
  
  # FIXME how to handle components ...
  
  vad_system_default(${ARGN} COMPONENTS C CXX)
  
  if (HDF5_FOUND)
    add_library(hdf5_cpp INTERFACE IMPORTED)
    set_target_properties(hdf5_cpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${HDF5_CXX_INCLUDE_DIRS}")
    set_target_properties(hdf5_cpp PROPERTIES INTERFACE_LINK_LIBRARIES "${HDF5_CXX_LIBRARIES}")
    make_imported_targets_global()
  else()
    # TODO good question should we abort or what?
  endif()
endfunction()

function(vad_live)
  message("run VAD_LIVE for HDF5")
  
  find_package(Perl REQUIRED)
  
  git_clone(HDF5)
  
  set(HDF5_GENERATE_HEADERS ON CACHE BOOL "Rebuild Generated Files")
  set(HDF5_BUILD_EXAMPLES OFF CACHE BOOL "Build HDF5 Library Examples")
  # TODO uhh ohh nameing clash in inevitable...
  set(BUILD_TESTING OFF CACHE BOOL "Build HDF5 Unit Testing")
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/HDF5" "${CMAKE_BINARY_DIR}/external/HDF5")
  
  set(HDF5_INCLUDE_DIR "${VAD_EXTERNAL_ROOT}/HDF5/c++/src;${VAD_EXTERNAL_ROOT}/HDF5/src;${CMAKE_BINARY_DIR}/external/HDF5" CACHE STRING "" FORCE)
  #set(HDF5_LIBRARIES "${VAD_EXTERNAL_ROOT}/HDF5/c++/src")
  #set(HDF5_LIBRARY_DIRS "${VAD_EXTERNAL_ROOT}/HDF5/c++/src")
  
  set_property(TARGET hdf5_cpp-static APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/HDF5")
  
  add_library(hdf5_cpp ALIAS hdf5_cpp-static)
  
  #add_library(HDF5::HDF5 INTERFACE IMPORTED)
  #set_target_properties(HDF5::HDF5 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${HDF5_INCLUDE_DIR}")
  #set_target_properties(HDF5::HDF5 PROPERTIES INTERFACE_LINK_LIBRARIES hdf5-static)
  
  # TODO should not be required...
  set(HDF5_FOUND TRUE CACHE INTERNAL "")
endfunction()
