# FIXME multiple urls? authentication?
#set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:light-field/metamat.git")
set(GIT_REPO "http://hci-repo.iwr.uni-heidelberg.de/light-field/metamat.git")

function(vad_system)
  vad_add_var(MetaMat_FOUND false)
endfunction()


function(vad_deps)
  vad_autodep_pkg(Boost "MetaMat")
  vad_autodep_pkg(HDF5 "MetaMat")
  vad_autodep_pkg(OpenCV "MetaMat")
endfunction()

function(vad_live)
  message("run VAD_LIVE for METAMAT")
  
  vad_deps(${ARGN})
  
  git_clone(MetaMat)
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/MetaMat" "${CMAKE_BINARY_DIR}/external/MetaMat")

  set_target_properties(metamat PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/MetaMat/include")
  
  set(MetaMat_FOUND true CACHE BOOL "" FORCE)
endfunction()
