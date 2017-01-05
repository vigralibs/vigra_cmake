# FIXME multiple urls? authentication?
set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:light-field/cliini.git")

function(vad_system)
  vad_system_default(${ARGN})
  if(CLIINI_FOUND AND NOT TARGET CLIINI::CLIINI)
    message(STATUS "Creating the CLIINI::CLIINI imported target.")
    add_library(CLIINI::CLIINI UNKNOWN IMPORTED)
    set_target_properties(CLIINI::CLIINI PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CLIINI_INCLUDE_DIRS}")
    # FIXME get absolute lib paths...
    #set_target_properties(CLIINI::CLIINI PROPERTIES INTERFACE_LINK_LIBRARIES "${METAMAT_LIBRARIES}")
    
    # FIXME search other libs?!!
    find_library(_imported_lib NAMES cliini HINTS ${CLIINI_LIBRARY_DIRS})
    set_property(TARGET CLIINI::CLIINI APPEND PROPERTY IMPORTED_LOCATION ${_imported_lib})
    
    make_imported_targets_global()
  endif()
endfunction()

function(vad_live)
  message("run VAD_LIVE for CLIINI")
  
  git_clone(cliini)
  
  if(WIN32)
    vigra_add_dep(fnmatch REQUIRED LIVE)
  endif()
  
  # TODO default seems to point to checkout out dir in source?!?
  add_subdirectory("${VAD_EXTERNAL_ROOT}/cliini" "${CMAKE_BINARY_DIR}/external/cliini")
  
  add_library(CLIINI::CLIINI INTERFACE IMPORTED)  
  
  set_target_properties(CLIINI::CLIINI PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CLIINI_INCLUDE_DIRS}")
  set_target_properties(CLIINI::CLIINI PROPERTIES INTERFACE_LINK_LIBRARIES cliini)

endfunction()
