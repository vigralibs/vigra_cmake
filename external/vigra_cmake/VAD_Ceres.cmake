include(VigraAddDep)

function(vad_system)
  vad_system_default(${ARGN})
  if(METAMAT_FOUND AND NOT TARGET ceres)
    message(ERROR "ceres found but no target \"ceres\"!")
  endif()
  
  add_library(CERES::CERES INTERFACE IMPORTED)  
  
  # ceres targets are, in fact, broken ... (missing includes)
  get_target_property(_CERES_INC ceres INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS}")
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_LINK_LIBRARIES "${CERES_LIBRARIES}")
  
endfunction()
