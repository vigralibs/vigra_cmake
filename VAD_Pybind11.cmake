include(VigraAddDep)

set(GIT_REPO "https://github.com/pybind/pybind11.git")

function(vad_live)
  vigra_add_dep(Python REQUIRED)
  git_clone(Pybind11)
  add_subdirectory("${VAD_EXTERNAL_ROOT}/Pybind11" "${VAD_EXTERNAL_ROOT}/Pybind11/build_external_dep")

  add_library(_VAD_Pybind11_STUB INTERFACE)
  set_property(TARGET _VAD_Pybind11_STUB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PYBIND11_INCLUDE_DIR})
  set_target_properties(_VAD_Pybind11_STUB PROPERTIES INTERFACE_LINK_LIBRARIES Python::Python)
  add_library(Pybind11::Pybind11 ALIAS _VAD_Pybind11_STUB)
endfunction()
