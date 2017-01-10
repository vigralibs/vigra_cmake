list(APPEND _VAD_AUTODEP_PROJECT_LIST "${CMAKE_PROJECT_NAME}")

if(VAD_HendriksWrappers_Included)
    return()
endif()

# Mark as included.
set(VAD_HendriksWrappers_Included YES)

set(_TARGET_PACKAGE_LIST cliini;cliini;METAMAT::METAMAT;MetaMat;CERES::CERES;Ceres;HDF5::HDF5;HDF5;OPENCV::OPENCV;OpenCV;boost_filesystem;Boost)

# TODO check system dependency!
function(vad_autodep_pkg _PKG_NAME _REQ_NAME)
  if (VAD_BUILD_${_PKG_NAME}_FROM_GIT)
    vigra_add_dep(${_PKG_NAME} LIVE)
  else()
    list(FIND _VAD_AUTODEP_MISSING_PKGS ${_PKG_NAME} _IDX_PKG_NAME)
    if (_IDX_PKG_NAME EQUAL -1)
      set(_VAD_AUTODEP_MISSING_PKGS "${_VAD_AUTODEP_MISSING_PKGS};${_PKG_NAME}" CACHE STRING "" FORCE)
      set(_VAD_AUTODEP_REQ_${_PKG_NAME} "${_REQ_NAME}" CACHE STRING "" FORCE)
      message("${_PKG_NAME} requested by ${_REQ_NAME}")
    endif()
  endif()
endfunction()

# FIXME for testing ... only PUBLIC, no Debug/Release, etc...
function(vad_link TARGT)
  foreach(L ${ARGN})
    if(TARGET ${L})
      target_link_libraries(${TARGT} ${ARGN})
    else()
      list(FIND _TARGET_PACKAGE_LIST ${L} _IDX_TGT)
      if (_IDX_TGT EQUAL -1)
        message(FATAL_ERROR "target ${L} does no exist and no corresponding package was found in")
      endif()
      math(EXPR _IDX_PKG "${_IDX_TGT} + 1")
      list(GET _TARGET_PACKAGE_LIST ${_IDX_PKG} _PKG_NAME)
      
      vad_autodep_pkg(${_PKG_NAME} ${CMAKE_PROJECT_NAME})
      target_link_libraries(${TARGT} ${L})
    endif()
  endforeach()
endfunction()

function(vad_auto_deps_show)
  list(FIND _VAD_AUTODEP_PROJECT_LIST ${CMAKE_PROJECT_NAME} IDX)
  if(IDX EQUAL -1)
    message(FATAL_ERROR "could not find this project ${CMAKE_PROJECT_NAME} in _VAD_AUTODEP_PROJECT_LIST, did you include VAD_HendriksWrappers.cmake?")
  endif()
  list(REMOVE_ITEM _VAD_AUTODEP_PROJECT_LIST ${CMAKE_PROJECT_NAME})
  if (NOT _VAD_AUTODEP_PROJECT_LIST)
    #recursively iterate all dependencies
    
    foreach(PKG ${_VAD_AUTODEP_MISSING_PKGS})
      # FIXME this is a direct copy, intergrate...
      find_file(VAD_${PKG}_FILE VAD_${PKG}.cmake ${CMAKE_MODULE_PATH})
      if(VAD_${PKG}_FILE)
        message(STATUS "VAD file 'VAD_${PKG}.cmake' was found at '${VAD_${PKG}_FILE}'. The VAD file will now be parsed.")
        vad_reset_hooks()
        include(${VAD_${PKG}_FILE})
        vad_deps(${PKG})
        vad_reset_hooks()
      else()
        message("Ohohhh no file  ${VAD_${PKG}_FILE} found!")
      endif()
    endforeach()
  
    message("missings PKGs: ")
    foreach(PKG ${_VAD_AUTODEP_MISSING_PKGS})
      message("  - ${PKG} (required by ${_VAD_AUTODEP_REQ_${PKG}})")
      option(VAD_BUILD_${PKG}_FROM_GIT "integrate LIVE source into project" off)
    endforeach()
    if (_VAD_AUTODEP_MISSING_PKGS)
      message("miss: ${_VAD_AUTODEP_MISSING_PKGS}")
      
      #cleanup
      set(_VAD_AUTODEP_MISSING_PKGS "" CACHE STRING "" FORCE)
      
      message(FATAL_ERROR "unfullfilled dependencies - either install and point cmake to the missing depdendencies or use VAD_BUILD_***_FROM_GIT to integrate the respective package into the project.")
    endif()
  endif()
  
  #cleanup
  set(_VAD_AUTODEP_MISSING_PKGS "" CACHE STRING "" FORCE)
endfunction()
