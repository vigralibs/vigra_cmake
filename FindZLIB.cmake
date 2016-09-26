include(VigraAddDep)

if(VAD_DEP_ZLIB_SATISFIED)
  message(STATUS "The ZLIB dependency has already been satisfied via VigraAddDep.")
else()
  list(REMOVE_ITEM CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
  find_package(ZLIB)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
endif()
