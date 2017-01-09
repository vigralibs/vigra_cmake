set(GIT_REPO "https://github.com/boostorg/system.git")

function(vad_live)
  message("run VAD_LIVE for boost_system")
  
  git_clone(boost_system)

  # forward includes
  set(_FORWARD_DEPS boost_config boost_predef boost_assert boost_core)
  foreach(DEP ${_FORWARD_DEPS}) 
    get_target_property(_DEP_INC_SINGLE ${DEP} INTERFACE_INCLUDE_DIRECTORIES)
    list(APPEND _DEP_INC ${_DEP_INC_SINGLE})
  endforeach()
  
  
  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_system/src/*")
  
  add_library(boost_system ${_SRCS})
  target_include_directories(boost_system PUBLIC "${VAD_EXTERNAL_ROOT}/boost_system/include")
  target_include_directories(boost_system PUBLIC ${_DEP_INC})
  target_link_libraries(boost_system boost_config boost_predef boost_assert boost_core)
endfunction()
