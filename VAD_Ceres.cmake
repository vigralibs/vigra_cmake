#include(VigraAddDep)

set(GIT_REPO "https://ceres-solver.googlesource.com/ceres-solver")

function(vad_system)
  vad_system_default(${ARGN})
  if(CERES_FOUND AND NOT TARGET ceres)
    message(ERROR "ceres found but no target \"ceres\"!")
  endif()
  
  add_library(CERES::CERES INTERFACE IMPORTED)  
  
  # ceres targets are, in fact, broken ... (missing includes)
  get_target_property(_CERES_INC ceres INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS}")
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_LINK_LIBRARIES "${CERES_LIBRARIES}")
  
endfunction()

function(vad_live)
  message("run VAD_LIVE for CERES")
  
  # TODO how would an installation handle this?
  list(APPEND CMAKE_MODULE_PATH "${VAD_EXTERNAL_ROOT}/Ceres/cmake")
  
  #prerequisits
  vigra_add_dep(Eigen REQUIRED LIVE)
  
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
  
  set(EIGENSPARSE ON) #use Eigen (no additional deps required - but suitesparse should be faster!)
  set(MINIGLOG ON) #use miniglog
  set(EIGENSPARSE ON)
  set(BUILD_TESTING OFF)
  
  # TODO default seems to point to checkout out dir in source?!?
  add_subdirectory("${VAD_EXTERNAL_ROOT}/Ceres" "${CMAKE_BINARY_DIR}/external/Ceres")
  
  add_library(CERES::CERES INTERFACE IMPORTED)  
  
  
  get_target_property(_CERES_INC ceres INTERFACE_INCLUDE_DIRECTORIES)
  
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS};${VAD_EXTERNAL_ROOT}/Ceres/include;${VAD_EXTERNAL_ROOT}/Ceres/config;${VAD_EXTERNAL_ROOT}/Ceres/internal/ceres/miniglog;${EIGEN_INCLUDE_DIRS}")
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_LINK_LIBRARIES ceres)

endfunction()
