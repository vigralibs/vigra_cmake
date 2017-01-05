set(GIT_REPO "https://github.com/boostorg/predef.git")

function(vad_live)
  message("run VAD_LIVE for boost_predef")
  
  git_clone(boost_predef)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  add_library(boost_predef INTERFACE IMPORTED)
  set_target_properties(boost_predef PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_predef/include")
endfunction()
