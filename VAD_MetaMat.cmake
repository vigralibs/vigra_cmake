# FIXME multiple urls? authentication?
set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:light-field/metamat.git")

function(vad_system)
  vad_system_default(${ARGN})
  if(METAMAT_FOUND AND NOT TARGET METAMAT::METAMAT)
    message(STATUS "Creating the METAMAT::METAMAT imported target.")
    add_library(METAMAT::METAMAT UNKNOWN IMPORTED)
    set_target_properties(METAMAT::METAMAT PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${METAMAT_INCLUDE_DIRS}")
    # FIXME get absolute lib paths...
    #set_target_properties(METAMAT::METAMAT PROPERTIES INTERFACE_LINK_LIBRARIES "${METAMAT_LIBRARIES}")
    
    # FIXME search other libs?!!
    find_library(_metamat_imported_lib NAMES metamat HINTS ${METAMAT_LIBRARY_DIRS})
    set_property(TARGET METAMAT::METAMAT APPEND PROPERTY IMPORTED_LOCATION ${_metamat_imported_lib})
    
    make_imported_targets_global()
  endif()
endfunction()

function(vad_live)
  message("run VAD_LIVE for METAMAT")
  
  vigra_add_dep(HDF5 REQUIRED LIVE)
  vigra_add_dep(OpenCV REQUIRED LIVE)
  vigra_add_dep(Boost REQUIRED LIVE)
  
  git_clone(MetaMat)
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/MetaMat" "${CMAKE_BINARY_DIR}/external/MetaMat")
  
  add_library(METAMAT::METAMAT INTERFACE IMPORTED)  
  

  # FIXME shouldnt this be forwarded?
  get_target_property(_MM_INC_DIRS metamat INTERFACE_INCLUDE_DIRECTORIES)
  
  set_target_properties(METAMAT::METAMAT PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/MetaMat/include;${_MM_INC_DIRS}")
  set_target_properties(METAMAT::METAMAT PROPERTIES INTERFACE_LINK_LIBRARIES metamat)  
endfunction()
