#include(VigraAddDep)

set(GIT_REPO "https://ceres-solver.googlesource.com/ceres-solver")

function(vad_system)
  message("run VAD_SYSTEM for CERES")
  
  
  vad_autodep_pkg(Eigen3 "Ceres")
  
  # FIXME environment is not reset after this!
  vad_system_default(${ARGN})
  
  if (NOT TARGET ceres)
    message(FATAL_ERROR "expected target \"ceres\"")
  endif()
  
  #ceres target seems to be missing any INTERFACE_INCLUDE_DIRECTORIES
  
  
  
  # FIXME this is heck create a vad_get_actual_target property (or fix vad_make_imported_target_global to not use alias!)
  #add_dependencies(_VAD_ceres_STUB Eigen3::Eigen)
  
  #get_property(_int_inc TARGET _VAD_ceres_STUB PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  #message("ceres target interface includes:  ${_int_inc}")
  get_property(_int_inc TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  message("eigen target interface includes:  ${_int_inc}")
  set_property(TARGET _VAD_ceres_STUB_INTERFACE APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${_int_inc}")
  set_property(TARGET _VAD_ceres_STUB APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${_int_inc}")
  #message(FATAL_ERROR "ohohh")
endfunction()

function(vad_deps)
  vad_autodep_pkg(Eigen "Ceres")
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
  
  #add_library(CERES::CERES INTERFACE IMPORTED)  
  
  
  #get_target_property(_CERES_INC ceres INTERFACE_INCLUDE_DIRECTORIES)
  
  #set_target_properties(CERES::CERES PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS};${VAD_EXTERNAL_ROOT}/Ceres/include;${VAD_EXTERNAL_ROOT}/Ceres/config;${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog;${EIGEN_INCLUDE_DIRS}")
  #set_target_properties(CERES::CERES PROPERTIES INTERFACE_LINK_LIBRARIES ceres)
  
  set(CERES_INCLUDE_DIRS "${CERES_INCLUDE_DIRS};${VAD_EXTERNAL_ROOT}/Ceres/include;${VAD_EXTERNAL_ROOT}/Ceres/config;${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog;${EIGEN_INCLUDE_DIRS}" CACHE STRING "" FORCE)
  set(CERES_LIBRARIES ceres CACHE STRING "" FORCE)
  set(CERES_FOUND TRUE CACHE BOOL "" FORCE)

endfunction()
