# FIXME multiple urls? authentication?
set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:light-field/fnmatch.git")

function(vad_live)
  message("run VAD_LIVE for FNMATCH")
  
  git_clone(fnmatch)
  
  # TODO default seems to point to checkout out dir in source?!?
  add_subdirectory("${VAD_EXTERNAL_ROOT}/fnmatch" "${CMAKE_BINARY_DIR}/external/fnmatch")
  
  add_library(FNMATCH::FNMATCH INTERFACE IMPORTED)  
  
  set_target_properties(FNMATCH::FNMATCH PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/fnmatch")
  set_target_properties(FNMATCH::FNMATCH PROPERTIES INTERFACE_LINK_LIBRARIES fnmatch)

endfunction()
