set(GIT_REPO "https://github.com/boostorg/smart_ptr.git")

function(vad_live)
  message("run VAD_LIVE for boost_smart_ptr")
  
  git_clone(boost_smart_ptr)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  
  get_target_property(_BOOST_SMARTPTR_EXTRA_INC_SINGLE boost_throw_exception INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND _BOOST_SMARTPTR_EXTRA_INC ${_BOOST_SMARTPTR_EXTRA_INC_SINGLE})
  
  add_library(boost_smart_ptr INTERFACE IMPORTED)
  set_target_properties(boost_smart_ptr PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_smart_ptr/include;${_BOOST_SMARTPTR_EXTRA_INC}")
endfunction()
