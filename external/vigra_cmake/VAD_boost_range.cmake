set(GIT_REPO "https://github.com/boostorg/range.git")

function(vad_live)
  message("run VAD_LIVE for boost_range")
  
  git_clone(boost_range)
  
  add_library(boost_range INTERFACE IMPORTED)
  set_target_properties(boost_range PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_range/include")
endfunction()
