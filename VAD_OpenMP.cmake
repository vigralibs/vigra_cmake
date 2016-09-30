function(vad_system NAME)
  # Call the standard FindOpenMP module.
  list(REMOVE_ITEM CMAKE_MODULE_PATH "${VAD_CMAKE_ROOT}")
  find_package(OpenMP)
  list(APPEND CMAKE_MODULE_PATH "${VAD_CMAKE_ROOT}")

  if(NOT OPENMP_FOUND)
    set(VAD_OpenMP_SYSTEM_NOT_FOUND TRUE CACHE INTERNAL "")
    return()
  endif()

  set(OpenMP_C_FLAGS "${OpenMP_C_FLAGS}" CACHE STRING "")
  set(OpenMP_CXX_FLAGS "${OpenMP_CXX_FLAGS}" CACHE STRING "")
  set(OpenMP_Fortran_FLAGS "${OpenMP_Fortran_FLAGS}" CACHE STRING "")
  mark_as_advanced(FORCE OpenMP_C_FLAGS)
  mark_as_advanced(FORCE OpenMP_CXX_FLAGS)
  mark_as_advanced(FORCE OpenMP_Fortran_FLAGS)
endfunction()
