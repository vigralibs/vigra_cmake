cmake_minimum_required(VERSION 2.6)
project(ucalib)
string(TOUPPER ${PROJECT_NAME} PNU)

#remove old local projectconfig
dep_lists_init()

#####################################################
## DEPENDENCIES
#####################################################
#metamat
dep_lists_append(MetaMat)
dep_lists_append(mm-mesh)
dep_lists_append(OpenCV OpenCV_INCLUDE_DIRS "" OpenCV_LIBS)
dep_lists_append(Ceres)
dep_lists_append(Alglib PRIVATE)

#####################################################
## COMPILER / FLAGS
#####################################################
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(${PNU}_BUILD_TYPE "SHARED")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
  set(${PNU}_BUILD_TYPE "SHARED")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fno-omit-frame-pointer -fPIC -std=c++11 -Wall -march=native -fopenmp")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  set(${PNU}_BUILD_TYPE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /noOy /W2 /EHsc")
endif()

# search pkgs
dep_lists_pkg_search()

#####################################################
## SET INCLUDES, LIBS, ... (public)
#####################################################
dep_lists_prepare_env()




#####################################################
## COMPILE AND LINK
#####################################################

#ucalib
add_library(ucalib SHARED proxy.cpp ucalib.cpp)
target_link_libraries(ucalib PUBLIC ${${PNU}_LIB})
target_link_libraries(ucalib PRIVATE ${${PNU}_PRIVATE_LIB})

list(APPEND ${PNU}_EXPORT_LIBS ucalib)

#####################################################
## PUBLIC HEADERS
#####################################################
set(${PNU}_HEADERS proxy.hpp ucalib.hpp)

#copy headers and generate XXXConfig.cmake
dep_lists_export_local()

#####################################################
## INSTALL
#####################################################

#install bins
install(TARGETS ${${PNU}_EXPORT_BINS}
            RUNTIME DESTINATION bin)
#install libs & export as export ${PROJECT_NAME}
install(TARGETS ${${PNU}_EXPORT_LIBS}
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
#headers
install(FILES ${${PNU}_PROJECT_HEADERS} DESTINATION include/${PROJECT_NAME})


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
install(FILES ${${PNU}_FULLPATH_HEADERS} DESTINATION include/${PROJECT_NAME} )

#local export into binary dir (may be found with find_package() using cmake .. -DCMAKE_PREFIX_PATH=<...> or with cmake-gui)
# if (${CMAKE_VERSION} VERSION_LESS 3.0.0)
#   #TODO use a generated list of targets!
#   export(TARGETS ucalib FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.cmake)
# else()
#   export(EXPORT ${PROJECT_NAME} FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.cmake)
# endif()

#export into cmakes local package cache - should be directly visible to cmake of local user
