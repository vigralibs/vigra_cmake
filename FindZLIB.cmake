include(VigraAddDep)

if(VAD_DEP_ZLIB_SATISFIED)
  message(STATUS "The ZLIB dependency has already been satisfied via VigraAddDep.")
else()
  vigra_add_dep(ZLIB)
endif()
