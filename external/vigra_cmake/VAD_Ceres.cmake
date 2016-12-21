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
  find_package(Eigen REQUIRED)
  
  git_clone(Ceres)
  
  #patch CMakeLists to work under a add_subdir
  file(READ "${VAD_EXTERNAL_ROOT}/Ceres/CMakeLists.txt" FILECONTENT)
  string(REPLACE "CMAKE_SOURCE_DIR" "CMAKE_CURRENT_LIST_DIR" FILECONTENT ${FILECONTENT})
  string(REPLACE "CMAKE_BINARY_DIR" "CMAKE_CURRENT_BINARY_DIR" FILECONTENT ${FILECONTENT})
  file(WRITE "${VAD_EXTERNAL_ROOT}/Ceres/CMakeLists.txt" ${FILECONTENT})
  
  message("output: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
  
  # TODO default seems to point to checkout out dir in source?!?
  add_subdirectory("${VAD_EXTERNAL_ROOT}/Ceres" "${CMAKE_BINARY_DIR}/external/Ceres")
  
  if(NOT TARGET ceres)
    message(ERROR "ceres found but no target \"ceres\"!")
  endif()
  
  add_library(CERES::CERES INTERFACE IMPORTED)  
  
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/Ceres/include;${VAD_EXTERNAL_ROOT}/Ceres/config;${EIGEN_INCLUDE_DIRS}")
  set_target_properties(CERES::CERES PROPERTIES INTERFACE_LINK_LIBRARIES ceres)

endfunction()
