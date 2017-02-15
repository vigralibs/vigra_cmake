set(GIT_REPO "https://github.com/boostorg/preprocessor.git")

function(vad_live)
  message("run VAD_LIVE for boost_preprocessor")
  
  git_clone(boost_preprocessor)
  
  add_library(boost_preprocessor INTERFACE IMPORTED)
  set_target_properties(boost_preprocessor PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_preprocessor/include")
endfunction()
