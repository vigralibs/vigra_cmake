set(GIT_REPO "https://github.com/live-clones/hdf5.git")

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
  
  # TODO should not be required...
  set(HDF5_FOUND TRUE CACHE INTERNAL "")
endfunction()
