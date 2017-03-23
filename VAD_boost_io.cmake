set(GIT_REPO "https://github.com/boostorg/io.git")

function(vad_live)
  message("run VAD_LIVE for boost_io")
  
  git_clone(boost_io)
  
  add_library(boost_io INTERFACE IMPORTED)
  set_target_properties(boost_io PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_io/include")
endfunction()
