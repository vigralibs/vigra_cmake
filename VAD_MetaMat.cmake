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


function(vad_deps)
  vad_autodep_pkg(Boost "MetaMat")
  vad_autodep_pkg(HDF5 "MetaMat")
  vad_autodep_pkg(OpenCV "MetaMat")
endfunction()

#Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

# Convert command output into a CMake list
STRING(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
STRING(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")

function(print_properties)
    message ("CMAKE_PROPERTY_LIST = ${CMAKE_PROPERTY_LIST}")
endfunction(print_properties)

function(print_target_properties tgt)
    if(NOT TARGET ${tgt})
      message("There is no target named '${tgt}'")
      return()
    endif()

    foreach (prop ${CMAKE_PROPERTY_LIST})
        string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" prop ${prop})
        # message ("Checking ${prop}")
        get_property(propval TARGET ${tgt} PROPERTY ${prop} SET)
        if (propval)
            get_target_property(propval ${tgt} ${prop})
            message ("${tgt} ${prop} = ${propval}")
        endif()
    endforeach(prop)
endfunction(print_target_properties)


function(vad_live)
  message("run VAD_LIVE for METAMAT")
  
  vad_deps(${ARGN})
  
  git_clone(MetaMat)
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/MetaMat" "${CMAKE_BINARY_DIR}/external/MetaMat")
  
  add_library(METAMAT::METAMAT INTERFACE IMPORTED)  

  set_target_properties(METAMAT::METAMAT PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/MetaMat/include")
  set_target_properties(METAMAT::METAMAT PROPERTIES INTERFACE_LINK_LIBRARIES metamat)
  # FIXME forwarding not working automatically?
  forward_target_includes(METAMAT::METAMAT metamat)
endfunction()
