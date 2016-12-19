include(VigraAddDep)

set(GIT_REPO "https://github.com/dmlc/xgboost.git")

set(GIT_CLONE_OPTS "--recursive")

function(vad_live)
  # Clone and add the subdirectory.
  git_clone(XGBoost)
  add_subdirectory("${VAD_EXTERNAL_ROOT}/XGBoost" "${VAD_EXTERNAL_ROOT}/XGBoost/build_external_dep")

  add_library(_VAD_XGBoost_STUB INTERFACE)
  list(APPEND _XGBoost_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/XGBoost/include" "${VAD_EXTERNAL_ROOT}/XGBoost/dmlc-core/include" "${VAD_EXTERNAL_ROOT}/XGBoost/rabit/include")
  set_property(TARGET _VAD_XGBoost_STUB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${_XGBoost_INCLUDE_DIRS})
  set_property(TARGET _VAD_XGBoost_STUB PROPERTY INTERFACE_LINK_LIBRARIES rabit libxgboost dmlccore)

  add_library(XGBoost::XGBoost ALIAS _VAD_XGBoost_STUB)
endfunction()
