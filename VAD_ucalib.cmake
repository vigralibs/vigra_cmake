# FIXME multiple urls? authentication?
set(GIT_REPO "http://hci-repo.iwr.uni-heidelberg.de/light-field/ucalib.git")


function(vad_system)
  vad_add_var(ucalib_FOUND false)
endfunction()

function(vad_deps)
  message("CALLED VAD_DEPS for ucalib")
  vad_autodep_pkg(hdmarker "ucalib")
  vad_autodep_pkg(Ceres "ucalib")
  vad_autodep_pkg(MetaMat "ucalib")
  vad_autodep_pkg(cliini "ucalib")
endfunction()

function(vad_live)
  message("run VAD_LIVE for ucalib")
  
  vad_deps(${ARGN})
  
  git_clone(ucalib)
  
  # TODO default seems to point to checkout out dir in source?!?
  message("add subdir: ${VAD_EXTERNAL_ROOT}/ucalib")
  add_subdirectory("${VAD_EXTERNAL_ROOT}/ucalib" "${CMAKE_BINARY_DIR}/external/ucalib")
  
  # for the included target
  set_target_properties(ucalib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/ucalib/include")

  set(ucalib_FOUND true CACHE BOOL "" FORCE)
endfunction()
