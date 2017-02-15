# FIXME multiple urls? authentication?
#set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:light-field/metamat.git")
set(GIT_REPO "http://hci-repo.iwr.uni-heidelberg.de/light-field/mm-mesh.git")

function(vad_deps)
  vad_autodep_pkg(MetaMat "mm-mesh")
endfunction()

function(vad_live)
  message("run VAD_LIVE for mm-mesh")
  
  vad_deps(${ARGN})
  
  git_clone(mm-mesh)
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/mm-mesh" "${CMAKE_BINARY_DIR}/external/mm-mesh")

  set_target_properties(mm-mesh PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/mm-mesh/include")
  
  set(MetaMat_FOUND true CACHE BOOL "" FORCE)
endfunction()
