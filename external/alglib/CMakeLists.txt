cmake_minimum_required (VERSION 2.8.8)
project(Alglib)

string(TOUPPER ${PROJECT_NAME} PNU)

# FIXME handle common includes
include(CMakePackageConfigListHelpers)
include(FlexDepLists)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

# FIXME add Debug, etc. on win!
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endforeach()

set(${PNU}_BUILD_TYPE "SHARED")
if (${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
  # using Clang
  list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
  #FIXME
  add_definitions(-DFNM_EXTMATCH=0)
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
  # using GCC
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fno-omit-frame-pointer -fPIC -std=c++11 -march=native")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  # using Visual Studio C++
  set(${PNU}_BUILD_TYPE "STATIC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /noOy /W2 /EHsc")
endif()

if (${PNU}_BUILD_TYPE STREQUAL "STATIC")
  set(BUILD_SHARED_LIBS OFF)
else()
  set(BUILD_SHARED_LIBS ON)
endif()

#####################################################
## DEPENDENCIES
#####################################################
dep_lists_init()
dep_lists_pkg_search()

file(DOWNLOAD http://www.alglib.net/translator/re/alglib-3.10.0.cpp.gpl.tgz "${CMAKE_CURRENT_BINARY_DIR}/alglib.tgz")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src")
execute_process(COMMAND ${CMAKE_COMMAND} -E tar xf "${CMAKE_CURRENT_BINARY_DIR}/alglib.tgz"
WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/src")

file(GLOB ${PNU}_SRCS "${CMAKE_CURRENT_BINARY_DIR}/src/cpp/src/*.cpp")
file(GLOB ${PNU}_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/src/cpp/src/*.h")

dep_lists_prepare_env()

#message("source: ")

#####################################################
## COMPILE
#####################################################
# headers are included so they form a dependency for file(COPY later)
add_library(Alglib ${${PNU}_SRCS})

set(${PNU}_EXPORT_LIBS Alglib)

#####################################################
## EXPORT
#####################################################

#copy headers and generate XXXConfig.cmake
dep_lists_export_local(DIRECT_HEADER_INCLUDE)



