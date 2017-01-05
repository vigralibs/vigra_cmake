set(GIT_REPO "https://github.com/boostorg/throw_exception.git")

function(vad_live)
  message("run VAD_LIVE for boost_throw_exception")
  
  git_clone(boost_throw_exception)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  
  add_library(boost_throw_exception INTERFACE IMPORTED)
  set_target_properties(boost_throw_exception PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_throw_exception/include")
endfunction()
