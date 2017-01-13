#include(VigraAddDep)

set(GIT_REPO "https://github.com/RLovelett/eigen.git")

function(vad_system)
  message("run VAD_SYSTEM for EIGEN")
  vad_system_default(${ARGN})
  
  if (EIGEN3_FOUND)
    set(EIGEN_FOUND true PARENT_SCOPE)
  endif()
  message("finshed run VAD_SYSTEM for EIGEN")
endfunction()

# FIXME this is a hack - to get EIGEN_INCLUDE_DIRS into the scope of the calling ceres CMakeLists.txt (use CACHE instead?)
function(vad_live)
  message("run VAD_LIVE for EIGEN")
  
  git_clone(Eigen3)
  
  add_library(Eigen3::Eigen INTERFACE IMPORTED)  
  
  #set include dir for variable based path handling (e.g. ceres-solver)
  # FIXME does VAD handle these vars?
  set(EIGEN_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/Eigen" CACHE STRING "" FORCE)
  
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}")
  
  # FIXME HACK
  set(EIGEN_VERSION "3.3.1" CACHE STRING "" FORCE)
  set(EIGEN_FOUND TRUE CACHE INTERNAL "")
  set(EIGEN3_FOUND TRUE CACHE INTERNAL "")
endfunction()
