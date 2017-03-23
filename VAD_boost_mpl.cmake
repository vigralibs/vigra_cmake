set(GIT_REPO "https://github.com/boostorg/mpl.git")

function(vad_live)
  message("run VAD_LIVE for boost_mpl")
  
  git_clone(boost_mpl)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  
  
  #dependent includes (TODO specify with add_dependency or the like (not workging...))
  get_target_property(_PRE_INC boost_preprocessor INTERFACE_INCLUDE_DIRECTORIES)
  
  add_library(boost_mpl INTERFACE IMPORTED)
  set_target_properties(boost_mpl PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_mpl/include;${_PRE_INC}")
endfunction()
