set(GIT_REPO "https://github.com/boostorg/config.git")

function(vad_live)
  message("run VAD_LIVE for boost_config")
  
  git_clone(boost_config)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  add_library(boost_config INTERFACE IMPORTED)
  set_target_properties(boost_config PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_config/include")
endfunction()
