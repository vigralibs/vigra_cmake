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
dep_lists_append(OpenCV OpenCV_INCLUDE_DIRS "" OpenCV_LIBS)
dep_lists_append(Ceres)

dep_lists_append(mm-mesh OPTIONAL)
dep_lists_append(Alglib PRIVATE OPTIONAL)

option(UCALIB_INTERNAL_ALGLIB "internal alglib build" FALSE)

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

if (NOT ${PNU}_WITH_ALGLIB)
  if (NOT GPL_BUILD)
    message(CRITICAL "Alglib not found - enable GPL_BUILD to build GPL'ed binaries using internal Alglib")
  else()
    set(UCALIB_INTERNAL_ALGLIB TRUE)
  endif()
endif()

if(UCALIB_INTERNAL_ALGLIB)
  message("using internal Alglib - binaries must be distributed GPL compliant!")
  add_subdirectory(external/alglib)
  # FIXME hack - use something lik dep_lists_check_package() to check wether alglib is available?
  set(FDP_Alglib_SEARCHED "")
  dep_lists_append(Alglib PRIVATE)
  dep_lists_pkg_search()
endif()

#####################################################
## SET INCLUDES, LIBS, ... (public)
#####################################################
set(${PNU}_HEADERS proxy.hpp ucalib.hpp)
dep_lists_prepare_env()




#####################################################
## COMPILE AND LINK
#####################################################

#ucalib
add_library(ucalib proxy.cpp ucalib.cpp)
target_link_libraries(ucalib PUBLIC ${${PNU}_LIB})
target_link_libraries(ucalib PRIVATE ${${PNU}_PRIVATE_LIB})

list(APPEND ${PNU}_EXPORT_LIBS ucalib)

#copy headers and generate XXXConfig.cmake
dep_lists_export_local()
