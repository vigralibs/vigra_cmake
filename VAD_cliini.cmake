# FIXME multiple urls? authentication?
#set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:light-field/cliini.git")
set(GIT_REPO "http://hci-repo.iwr.uni-heidelberg.de/light-field/cliini.git")

function(vad_system)
  vad_add_var(cliini_FOUND false)
endfunction()

function(vad_live)
  message("run VAD_LIVE for CLIINI")
  
  git_clone(cliini)
  
  if(WIN32)
    vigra_add_dep(fnmatch REQUIRED LIVE)
  endif()
  
  # TODO default seems to point to checkout out dir in source?!?
  add_subdirectory("${VAD_EXTERNAL_ROOT}/cliini" "${CMAKE_BINARY_DIR}/external/cliini")
  
  vad_add_var(cliini_FOUND true)
  
  # for the included target
  #set_target_properties(cliini PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/cliini/include")
  #set_target_properties(cliini-cpp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/cliini/include")

endfunction()
