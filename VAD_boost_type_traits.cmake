set(GIT_REPO "https://github.com/boostorg/type_traits.git")

function(vad_live)
  message("run VAD_LIVE for boost_type_traits")
  
  git_clone(boost_type_traits)

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  add_library(boost_type_traits INTERFACE IMPORTED)
  set_target_properties(boost_type_traits PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_type_traits/include")
endfunction()
