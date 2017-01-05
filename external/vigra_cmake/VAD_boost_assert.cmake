set(GIT_REPO "https://github.com/boostorg/assert.git")

function(vad_live)
  message("run VAD_LIVE for boost_assert")
  
  git_clone(boost_assert)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  add_library(boost_assert INTERFACE IMPORTED)
  set_target_properties(boost_assert PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_assert/include")
endfunction()
