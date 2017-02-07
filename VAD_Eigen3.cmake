#include(VigraAddDep)

set(GIT_REPO "https://github.com/RLovelett/eigen.git")

function(vad_system)
  message("run VAD_SYSTEM for EIGEN3")
  find_package_plus_no_import(${ARGN} NO_CMAKE_BUILDS_PATH)
  
  # ceres uses it's own findEigen routine... and expexts EIGEN_... as var names...
  if (NOT EIGEN_FOUND AND EIGEN3_FOUND)
    vad_add_var(EIGEN_FOUND ${EIGEN3_FOUND})
  endif()
  if (NOT EIGEN_VERSION AND Eigen3_VERSION)
    vad_add_var(EIGEN_VERSION ${Eigen3_VERSION})
  endif()
  if (NOT EIGEN_INCLUDE_DIRS AND EIGEN3_INCLUDE_DIRS)
    vad_add_var(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
  endif()
  make_imported_targets_global()
endfunction()

function(vad_live)
  message("run VAD_LIVE for EIGEN")
  
  git_clone(Eigen3)
  
  add_library(Eigen3::Eigen INTERFACE IMPORTED)  
  
  #set include dir for variable based path handling (e.g. ceres-solver)
  # FIXME does VAD handle these vars?
  set(EIGEN_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/Eigen3" CACHE STRING "" FORCE)
  
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}")
  
  # FIXME HACK
  set(EIGEN_VERSION "3.3.1" CACHE STRING "" FORCE)
  set(EIGEN_FOUND TRUE CACHE INTERNAL "")
  set(EIGEN3_FOUND TRUE CACHE INTERNAL "")
endfunction()
