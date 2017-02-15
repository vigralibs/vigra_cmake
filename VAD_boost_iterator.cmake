set(GIT_REPO "https://github.com/boostorg/iterator.git")

function(vad_live)
  message("run VAD_LIVE for boost_iterator")
  
  git_clone(boost_iterator)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  
  #dependent includes (TODO specify with add_dependency or the like (not workging...))
  get_target_property(_BOOST_ITER_EXTRA_INC_SINGLE boost_mpl INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND _BOOST_ITER_EXTRA_INC ${_BOOST_ITER_EXTRA_INC_SINGLE})
  get_target_property(_BOOST_ITER_EXTRA_INC_SINGLE boost_static_assert INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND _BOOST_ITER_EXTRA_INC ${_BOOST_ITER_EXTRA_INC_SINGLE})
  get_target_property(_BOOST_ITER_EXTRA_INC_SINGLE boost_detail INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND _BOOST_ITER_EXTRA_INC ${_BOOST_ITER_EXTRA_INC_SINGLE})
  
  add_library(boost_iterator INTERFACE IMPORTED)
  set_target_properties(boost_iterator PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_iterator/include;${_BOOST_ITER_EXTRA_INC}")
endfunction()
