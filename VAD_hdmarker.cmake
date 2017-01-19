# FIXME multiple urls? authentication?
set(GIT_REPO "git@hci-repo.iwr.uni-heidelberg.de:hsiedelm/hdmarker.git")

function(vad_system)
  vad_system_default(${ARGN} NO_CMAKE_BUILDS_PATH)
endfunction()

function(vad_deps)
  vad_autodep_pkg(OpenCV "hdmarker")
  vad_autodep_pkg(Ceres "hdmarker")
endfunction()

function(vad_live)
  message("run VAD_LIVE for hdmarker")
  
  vad_deps(${ARGN})
  
  if (NOT OPENCV_FOUND AND NOT CERES_FOUND)
    message(FATAL_ERROR "hdmarker: OpenCV or Ceres missing!")
    return()
  endif()
  
  git_clone(hdmarker)
  
  add_subdirectory("${VAD_EXTERNAL_ROOT}/hdmarker" "${CMAKE_BINARY_DIR}/external/hdmarker")
  
  set_target_properties(hdmarker PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/external/hdmarker/include")
  
endfunction()
