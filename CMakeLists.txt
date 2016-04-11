cmake_minimum_required(VERSION 2.6)
project(ucalib)
string(TOUPPER ${PROJECT_NAME} P_N_U)

### hack / temporary
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/alglib)

#####################################################
## VARIABLE DOCUMENTATION
#####################################################
#Meta:
#  P_N_U              - project name in uppercase - prefixed to (nearly) all variables
#
#Dependencies (packages and respective cmake variables)
#  _PKG               - required cmake package names - passed to find_package() - will also be included from ...Config.cmake
#  _PKG_INC           - varable names to add to include_directories (from the respecive find_package() calls)
#  _PKG_LIB           - varable names to add to link_libraries
#
#actual directories/libraries from dependencies
#  _DEPENDS_INC       - actual content from _PKG_INC vars
#  _DEPENDS_LINK      - 
#  _DEPENDS_LIB       - 
#
#  _EXPORT_LIBS       - targets (libraries) for installation, export and to put into _LIBRARIES into ..Config.cmake
#  _EXPORT_BINS       - targets (executables) to install
#
#
#  _BUILD_TYPE        - SHARED or "" (set depending on compiler)
#
#  _PROJECT_HEADERS
#  _PROJECT_LIBRARIES

#####################################################
## DEPENDENCIES
#####################################################
#opencv
list(APPEND ${P_N_U}_PKG OpenCV)
list(APPEND ${P_N_U}_PKG_INC OpenCV_INCLUDE_DIRS)
list(APPEND ${P_N_U}_PKG_LIB OpenCV_LIBS)

#ceres
list(APPEND ${P_N_U}_PKG Ceres)
list(APPEND ${P_N_U}_PKG_INC CERES_INCLUDE_DIRS)
list(APPEND ${P_N_U}_PKG_LIB CERES_LIBRARIES)

#alglib
list(APPEND ${P_N_U}_PRIVATE_PKG Alglib)
list(APPEND ${P_N_U}_PRIVATE_PKG_INC ALGLIB_INCLUDE_DIRS)
list(APPEND ${P_N_U}_PRIVATE_PKG_LIB ALGLIB_LIBRARIES)

#####################################################
## COMPILER / FLAGS
#####################################################
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
  set(${P_N_U}_BUILD_TYPE "SHARED")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fno-omit-frame-pointer -fPIC -std=c++11 -Wall -march=native -fopenmp")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  set(${P_N_U}_BUILD_TYPE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /noOy /W2 /EHsc")
endif()

# search pkgs
dep_lists_pkg_search(${P_N_U})

#####################################################
## SET INCLUDES, LIBS, ... (public)
#####################################################
# foreach(PACKAGE ${${P_N_U}_PKG})
#   find_package(${PACKAGE} REQUIRED)
# endforeach()
foreach(INCLUDE ${${P_N_U}_PKG_INC})
  list(APPEND ${P_N_U}_DEPENDS_INC ${${INCLUDE}})
endforeach()
foreach(LIBDIR ${${P_N_U}_PKG_LINK})
  list(APPEND ${P_N_U}_DEPENDS_LINK ${${LIBDIR}})
endforeach()
foreach(LIB ${HDMARKER_PKG_LIB})
  list(APPEND ${P_N_U}_DEPENDS_LIB ${${LIB}})
endforeach()

#####################################################
## SET INCLUDES, LIBS, ... (private)
#####################################################

# foreach(PACKAGE ${${P_N_U}_PRIVATE_PKG})
#   find_package(${PACKAGE} REQUIRED)
# endforeach()
foreach(INCLUDE ${${P_N_U}_PRIVATE_PKG_INC})
  list(APPEND ${P_N_U}_PRIVATE_DEPENDS_INC ${${INCLUDE}})
endforeach()
foreach(LIBDIR ${${P_N_U}_PRIVATE_PKG_LINK})
  list(APPEND ${P_N_U}_PRIVATE_DEPENDS_LINK ${${LIBDIR}})
endforeach()
foreach(LIB ${${P_N_U}_PRIVATE_PKG_LIB})
  list(APPEND ${P_N_U}_PRIVATE_DEPENDS_LIB ${${LIB}})
endforeach()

#####################################################
## PUBLIC HEADERS
#####################################################
list(APPEND ${P_N_U}_PROJECT_HEADERS loess.hpp fitting.hpp corr_lines.hpp simplecloud.hpp cam_models.hpp settings.hpp gencam.hpp)
foreach(header ${${P_N_U}_PROJECT_HEADERS})
  get_filename_component(_FILE ${header} NAME)
  configure_file(${header} "include/${PROJECT_NAME}/${_FILE}" COPYONLY)
endforeach()

#####################################################
## DIRS (from above)
#####################################################
include_directories(${${P_N_U}_DEPENDS_INC} ${${P_N_U}_PRIVATE_DEPENDS_INC})
link_directories(${${P_N_U}_DEPENDS_LINK} ${${P_N_U}_PRIVATE_DEPENDS_LINK})


#####################################################
## COMPILE AND LINK
#####################################################

#ucalib
add_library(ucalib SHARED loess.cpp fitting.cpp corr_lines.cpp simplecloud.cpp common.cpp cam_models.cpp gencam.cpp)
target_link_libraries(ucalib ${${P_N_U}_DEPENDS_LIB} ${${P_N_U}_PRIVATE_DEPENDS_LIB})
list(APPEND ${P_N_U}_EXPORT_LIBS ucalib)


#####################################################
## INSTALL
#####################################################

#install bins
install(TARGETS ${${P_N_U}_EXPORT_BINS}
            EXPORT ${PROJECT_NAME}
            RUNTIME DESTINATION bin)
#install libs & export as export ${PROJECT_NAME}
install(TARGETS ${${P_N_U}_EXPORT_LIBS}
            EXPORT ${PROJECT_NAME}
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
#headers
install(FILES ${${P_N_U}_PROJECT_HEADERS} DESTINATION include/${PROJECT_NAME})


#####################################################
## EXPORT
#####################################################

#export helper
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
include(CMakePackageConfigListHelpers)

#####################################################
## ...Config.cmake generation
#####################################################
set(CMAKECONFIG_PKG ${${P_N_U}_PKG})
set(CMAKECONFIG_PKG_INC ${${P_N_U}_PKG_INC})
set(CMAKECONFIG_PKG_LINK ${${P_N_U}_PKG_LINK})
set(CMAKECONFIG_PKG_LIB ${${P_N_U}_PKG_LIB})

set(CMAKECONFIG_INC "include") #in build dir - headers were already copied earlier
set(CMAKECONFIG_LIB ${${P_N_U}_EXPORT_LIBS}) # our libs to link on import


#####################################################
## local config.cmake
#####################################################
set(CMAKECONFIG_CMAKE_DIR ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKECONFIG_LINK ${CMAKE_CURRENT_BINARY_DIR})

set(INSTALL_PREFIX_BACKUP ${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
configure_package_config_file(cmake/projectConfig.cmake.in
                              "${CMAKECONFIG_CMAKE_DIR}/${PROJECT_NAME}Config.cmake"
                              INSTALL_DESTINATION "${CMAKECONFIG_CMAKE_DIR}"
                              PATH_VARS CMAKECONFIG_PKG CMAKECONFIG_PKG_INC CMAKECONFIG_PKG_LINK CMAKECONFIG_PKG_LIB CMAKECONFIG_INC CMAKECONFIG_LINK CMAKECONFIG_LIB)
set(CMAKE_INSTALL_PREFIX ${INSTALL_PREFIX_BACKUP})


#####################################################
# install config.cmake
#####################################################
set(CMAKECONFIG_CMAKE_DIR "lib/cmake/${PROJECT_NAME}")

set(CMAKECONFIG_LINK "lib")

configure_package_config_file(cmake/projectConfig.cmake.in
                              "config/${PROJECT_NAME}Config.cmake"
                              INSTALL_DESTINATION "${CMAKECONFIG_CMAKE_DIR}"
                              PATH_VARS CMAKECONFIG_PKG CMAKECONFIG_PKG_INC CMAKECONFIG_PKG_LINK CMAKECONFIG_PKG_LIB CMAKECONFIG_INC CMAKECONFIG_LINK CMAKECONFIG_LIB)

#####################################################
# export targets and install config.cmake files
#####################################################
# Install the ...Config.cmake
install(FILES
  "${PROJECT_BINARY_DIR}/config/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKECONFIG_CMAKE_DIR})

#export targets for installation
install(EXPORT ${PROJECT_NAME} DESTINATION ${CMAKECONFIG_CMAKE_DIR})
install(FILES ${${P_N_U}_FULLPATH_HEADERS} DESTINATION include/${PROJECT_NAME} )

#local export into binary dir (may be found with find_package() using cmake .. -DCMAKE_PREFIX_PATH=<...> or with cmake-gui)
if (${CMAKE_VERSION} VERSION_LESS 3.0.0)
  #TODO use a generated list of targets!
  export(TARGETS ucalib FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.cmake)
else()
  export(EXPORT ${PROJECT_NAME} FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.cmake)
endif()

#export into cmakes local package cache - should be directly visible to cmake of local user
export(PACKAGE ${PROJECT_NAME})
