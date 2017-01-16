#include(VigraAddDep)

set(GIT_REPO "https://ceres-solver.googlesource.com/ceres-solver")

function(vad_system)
  message("run VAD_SYSTEM for CERES")
  
  
  vad_autodep_pkg(Eigen3 "Ceres")
  
  # FIXME environment is not reset after this!
  find_package_plus_no_import(${ARGN})
  
  if (NOT ceres_FOUND OR NOT TARGET ceres)
    # TODO add/use a vad_*** function
    set(VAD_ceres_SYSTEM_NOT_FOUND TRUE CACHE INTERNAL "")
    return()
  endif()
  
  set_property(TARGET ceres APPEND PROPERTY INTERFACE_LINK_LIBRARIES "Eigen3::Eigen")
  
  # TODO put this into a nice interface?
  make_imported_targets_global()
endfunction()

function(vad_deps)
  vad_autodep_pkg(Eigen3 "Ceres")
endfunction()

function(vad_live)
  message("run VAD_LIVE for CERES")
  
  # TODO how would an installation handle this?
  list(APPEND CMAKE_MODULE_PATH "${VAD_EXTERNAL_ROOT}/Ceres/cmake")
  
  #prerequisits
  vad_deps(${ARGN})
  
  git_clone(Ceres)
  
  #patch CMakeLists to work under a add_subdir
  file(READ "${VAD_EXTERNAL_ROOT}/Ceres/CMakeLists.txt" FILECONTENT)
  string(REPLACE "CMAKE_SOURCE_DIR" "CMAKE_CURRENT_LIST_DIR" FILECONTENT ${FILECONTENT})
  string(REPLACE "CMAKE_BINARY_DIR" "CMAKE_CURRENT_BINARY_DIR" FILECONTENT ${FILECONTENT})
  string(REPLACE "add_custom_target(uninstall\n" "add_custom_target(uninstall-ceres\n" FILECONTENT ${FILECONTENT})
  file(WRITE "${VAD_EXTERNAL_ROOT}/Ceres/CMakeLists.txt" ${FILECONTENT})
  
  #patch (HACK) logging.h
  file(READ "${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog/glog/logging.h" FILECONTENT)
  string(REPLACE "// Log severity level constants."
        "#undef ERROR\n#define MAX_LOG_LEVEL -1" FILECONTENT "${FILECONTENT}")
  file(WRITE "${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog/glog/logging.h" "${FILECONTENT}")  
  
  set(EIGENSPARSE ON CACHE BOOL "" FORCE) #use Eigen (no additional deps required - but suitesparse should be faster!)
  set(MINIGLOG ON CACHE BOOL "" FORCE) #use miniglog
  set(EIGENSPARSE ON CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  
  # TODO default seems to point to checkout out dir in source?!?
  add_subdirectory("${VAD_EXTERNAL_ROOT}/Ceres" "${CMAKE_BINARY_DIR}/external/Ceres")
  
  set_property(TARGET ceres APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/Ceres/include;${VAD_EXTERNAL_ROOT}/Ceres/config;${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog")
  target_link_libraries(ceres INTERFACE Eigen3::Eigen)
  
  set(CERES_INCLUDE_DIRS "${CERES_INCLUDE_DIRS};${VAD_EXTERNAL_ROOT}/Ceres/include;${VAD_EXTERNAL_ROOT}/Ceres/config;${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog;${EIGEN_INCLUDE_DIRS}" CACHE STRING "" FORCE)
  set(CERES_LIBRARIES ceres CACHE STRING "" FORCE)
  set(CERES_FOUND TRUE CACHE BOOL "" FORCE)

  #TODO fix this:
  add_library(ceres_hack3 ALIAS ceres)
  
endfunction()
