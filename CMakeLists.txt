cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

enable_testing()

project(vigra_imageio VERSION 0.1 LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vigra_cmake")
include(VigraAddExternal)

vigra_add_external(vigra_core REPO https://github.com/vigralibs/vigra_core.git)

add_library(vigra_imageio
    src/bmp.cxx
    src/byteorder.cxx
    #src/codecmanager.cxx
    src/compression.cxx
)

target_include_directories(vigra_imageio PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(vigra_imageio vigra_core)


#add_library(threadpool INTERFACE)
#target_link_libraries(threadpool INTERFACE core INTERFACE Threads::Threads)

#target_include_directories(threadpool INTERFACE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

#install(DIRECTORY include/ DESTINATION include)
#add_subdirectory(test)
