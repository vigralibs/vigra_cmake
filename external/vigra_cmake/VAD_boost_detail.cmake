set(GIT_REPO "https://github.com/boostorg/detail.git")

function(vad_live)
  message("run VAD_LIVE for boost_detail")
  
  git_clone(boost_detail)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  
  add_library(boost_detail INTERFACE IMPORTED)
  set_target_properties(boost_detail PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_detail/include")
endfunction()
