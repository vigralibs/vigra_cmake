include(VigraAddDep)

if(VAD_DEP_OpenMP_SATISFIED)
  message(STATUS "The OpenMP dependency has already been satisfied via VigraAddDep.")
else()
  vigra_add_dep(OpenMP)
endif()
