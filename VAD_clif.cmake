# FIXME multiple urls? authentication?
set(GIT_REPO "http://hci-repo.iwr.uni-heidelberg.de/light-field/clif.git")
set(GIT_CLONE_OPTS "-b;ucalib_rework")


function(vad_system)
  vad_add_var(ucalib_FOUND false)
endfunction()

function(vad_deps)
  message("CALLED VAD_DEPS for clif")
  vad_autodep_pkg(ucalib "clif")
endfunction()

function(vad_live)
  message("run VAD_LIVE for clif")
  
  vad_deps(${ARGN})
  
  git_clone(clif)
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/clif" "${CMAKE_BINARY_DIR}/external/clif")

  set(clif_FOUND true CACHE BOOL "" FORCE)
endfunction()
