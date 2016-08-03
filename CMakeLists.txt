cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

enable_testing()

project(vigra_imageio VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vigra_cmake")
include(VigraAddExternal)

vigra_add_external(vigra_core REPO https://github.com/vigralibs/vigra_core.git)

option(ENABLE_PNG "Build with PNG support.")
option(ENABLE_OPENEXR "Build with OpenEXR support.")

if(ENABLE_PNG)
  find_package(PNG REQUIRED)
endif()

if(ENABLE_OPENEXR)
  find_package(OpenEXR REQUIRED)
endif()

add_library(vigra_imageio
    src/bmp.cxx
    src/byteorder.cxx
    src/codecmanager.cxx
    src/compression.cxx
    src/exr.cxx
    src/gif.cxx
    src/hdr.cxx
    #src/hdf5impex.cxx
    #src/hdf5_rf_impex.cxx
    src/iccjpeg.c
    src/imageinfo.cxx
    src/jpeg.cxx
    src/lz4.c
    src/png.cxx
    src/pnm.cxx
    src/rgbe.c
    src/sifImport.cxx
    src/sun.cxx
    src/tiff.cxx
    src/viff.cxx
    src/void_vector.cxx
)

target_include_directories(vigra_imageio PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(vigra_imageio PUBLIC vigra_core)

if(ENABLE_PNG)
  target_compile_definitions(vigra_imageio PRIVATE HasPNG)
  target_link_libraries(vigra_imageio PRIVATE PNG::PNG)
endif()

if(ENABLE_OPENEXR)
  target_compile_definitions(vigra_imageio PRIVATE HasEXR)
  target_link_libraries(vigra_imageio PRIVATE OpenEXR::OpenEXR)
endif()

#add_library(threadpool INTERFACE)
#target_link_libraries(threadpool INTERFACE core INTERFACE Threads::Threads)

#target_include_directories(threadpool INTERFACE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

#install(DIRECTORY include/ DESTINATION include)
#add_subdirectory(test)
