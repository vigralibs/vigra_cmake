set(GIT_REPO "https://github.com/boostorg/static_assert.git")

function(vad_live)
  message("run VAD_LIVE for boost_static_assert")
  
  git_clone(boost_static_assert)
  
  add_library(boost_static_assert INTERFACE IMPORTED)
  set_target_properties(boost_static_assert PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_static_assert/include")
endfunction()
