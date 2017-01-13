cmake_minimum_required(VERSION 2.6)
project(ucalib)

#for 
cmake_policy(VERSION 2.8.12)


#####################################################
## INIT
#####################################################
include(external/flexdeplists/FlexDepLists.cmake)
dep_lists_init()

#set(VAD_VERBOSE true)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/vigra_cmake")
#set(VAD_VERBOSE TRUE)
# TODO integrate
include(VAD_HendriksWrappers)

# TODO integrate into VAD framework
include(VigraAddDep)


#####################################################
## ENVIRONMENT
#####################################################
option(${_PNU}_BUILD_LIBIGL "build with (internal) libigl" ON)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fno-omit-frame-pointer -fPIC -std=c++11 -march=native -fopenmp")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  #openmp for ceres solver
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /noOy /W2 /EHsc /openmp /MP")
endif()

# default to static on win shared on others
if (NOT DEFINED BUILD_SHARED_LIBS)
  if (WIN32)
    option(BUILD_SHARED_LIBS "build shared libraries" OFF)
  else()
    option(BUILD_SHARED_LIBS "build shared libraries" ON)
  endif()
endif()

#####################################################
## DEPENDENCIES
#####################################################

#dep_lists_append(mm-mesh OPTIONAL)

# this only exports the headers ... remove/only for install!
set(${_PNU}_HEADERS proxy.hpp ucalib.hpp loess.hpp)
dep_lists_prepare_env()

#####################################################
## COMPILE
#####################################################


add_library(ucalib proxy.cpp ucalib.cpp)
vad_link(ucalib cliini ceres_hack3 metamat)


add_executable(ucalib_sample sample.cpp)
vad_link(ucalib_sample ucalib hdmarker cliini-cpp)

vad_auto_deps_show()

