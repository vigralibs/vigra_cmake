set(GIT_REPO "https://github.com/boostorg/functional.git")

function(vad_live)
  message("run VAD_LIVE for boost_functional")
  
  git_clone(boost_functional)
  
  add_library(boost_functional INTERFACE IMPORTED)
  set_target_properties(boost_functional PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_functional/include")
endfunction()
