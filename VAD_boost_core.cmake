set(GIT_REPO "https://github.com/boostorg/core.git")

function(vad_live)
  message("run VAD_LIVE for boost_core")
  
  git_clone(boost_core)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  add_library(boost_core INTERFACE IMPORTED)
  set_target_properties(boost_core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_core/include")
endfunction()
