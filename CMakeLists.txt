cmake_minimum_required(VERSION 2.6)
project(ucalib)


#####################################################
## INIT
#####################################################
include(external/flexdeplists/FlexDepLists.cmake)
dep_lists_init()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/vigra_cmake")
#set(VAD_VERBOSE TRUE)
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /noOy /W2 /EHsc /openmp")
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
#metamat

#dep_lists_append(MetaMat)
vigra_add_dep(MetaMat REQUIRED GLOBAL_PUBLIC)

if (WIN32)
  # only required to import the respective targets... (we should export them ourselves?!?)
  set(SuiteSparse_USE_LAPACK_BLAS ON)
  set(BLAS_STATIC ON)
  # FIXME this should work (but will try to create BLAS target multiple times...)
  list(APPEND ${_PNU}_EXPORT_VARS SuiteSparse_USE_LAPACK_BLAS)
  list(APPEND ${_PNU}_EXPORT_VARS BLAS_STATIC)
  dep_lists_append(SuiteSparse)
endif()

vigra_add_dep(Ceres REQUIRED LIVE GLOBAL_PUBLIC)
#dep_lists_append(mm-mesh OPTIONAL)

# search pkgs
dep_lists_pkg_search()

set(${_PNU}_HEADERS proxy.hpp ucalib.hpp loess.hpp)
dep_lists_prepare_env()

#####################################################
## COMPILE
#####################################################
add_library(ucalib proxy.cpp ucalib.cpp)
target_link_libraries(ucalib PUBLIC ${${_PNU}_LIB} ${VAD_DEP_UCALIB_TARGETS})
target_link_libraries(ucalib PRIVATE ${${_PNU}_PRIVATE_LIB})
list(APPEND ${_PNU}_EXPORT_LIBS ucalib)

#####################################################
## INSTALL
#####################################################
# FIXME add dependend packages (/targets?!) from VAD...stuff
# FIXME add enable features (includes enabled dependencies in VAD...)
# FIXME export/install the used VAD files and 
dep_lists_export_local()

