include(VigraAddDep)

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
    
    #message("metamat lib: ${_metamat_imported_lib}")
    
    make_imported_targets_global()
  endif()
  
  get_target_property(DBG METAMAT::METAMAT INTERFACE_INCLUDE_DIRECTORIES)
  message("mm inc: ${DBG}")
endfunction()
