cmake_minimum_required(VERSION 2.6)
project(ucalib)


#####################################################
## INIT
#####################################################
include(../flexdeplists/FlexDepLists.cmake)
dep_lists_init()

#####################################################
## ENVIRONMENT
#####################################################
option(${PNU}_BUILD_LIBIGL "build with (internal) libigl" ON)
option(${PNU}_INTERNAL_ALGLIB "internal alglib build" OFF)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fno-omit-frame-pointer -fPIC -std=c++11 -march=native")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /noOy /W2 /EHsc")
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
dep_lists_append(MetaMat)
dep_lists_append(OpenCV OpenCV_INCLUDE_DIRS "" OpenCV_LIBS)
dep_lists_append(Ceres)

dep_lists_append(mm-mesh OPTIONAL)
# FIXME should also work as PRIVATE (but doesn't with MSVC in Debug, something is exported from alglib in ucalib?!)
dep_lists_append(Alglib OPTIONAL)

# search pkgs
dep_lists_pkg_search()

if (NOT ${PNU}_WITH_ALGLIB)
  set(UCALIB_INTERNAL_ALGLIB TRUE)
endif()

if (UCALIB_INTERNAL_ALGLIB)
  add_subdirectory(external/alglib)
  # FIXME remove alglib from optional?!
  dep_lists_append(Alglib)
  dep_lists_pkg_recheck(Alglib)
endif()

set(${PNU}_HEADERS proxy.hpp ucalib.hpp)
dep_lists_prepare_env()

#####################################################
## COMPILE
#####################################################
add_library(ucalib proxy.cpp ucalib.cpp)
target_link_libraries(ucalib PUBLIC ${${PNU}_LIB})
target_link_libraries(ucalib PRIVATE ${${PNU}_PRIVATE_LIB})
list(APPEND ${PNU}_EXPORT_LIBS ucalib)

#####################################################
## INSTALL
#####################################################
dep_lists_export_local()

#####################################################
## INFO
#####################################################
message("build info for ${PROJECT_NAME}:")
message("  optional components:")
msg_yesno("    internal alglib" UCALIB_INTERNAL_ALGLIB COLOR NONE)
message("")
