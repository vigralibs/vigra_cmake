include(VigraAddDep)

function(vad_system)
  # Call the standard FindOpenMP module.
  find_package_orig(OpenMP)

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
