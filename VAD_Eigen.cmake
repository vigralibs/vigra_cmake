#include(VigraAddDep)

set(GIT_REPO "https://github.com/RLovelett/eigen.git")

function(vad_system)
  vad_system_default(${ARGN})
endfunction()

# FIXME this is a hack - to get EIGEN_INCLUDE_DIRS into the scope of the calling ceres CMakeLists.txt (use CACHE instead?)
macro(vad_live)
  message("run VAD_LIVE for EIGEN")
  
  git_clone(Eigen)
  
  add_library(EIGEN::EIGEN INTERFACE IMPORTED)  
  
  #set include dir for variable based path handling (e.g. ceres-solver)
  set(EIGEN_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/Eigen" PARENT_SCOPE)
  
  set_target_properties(EIGEN::EIGEN PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}")
  
  message("eigen inc dirs: ${EIGEN_INCLUDE_DIRS}")
endmacro()
