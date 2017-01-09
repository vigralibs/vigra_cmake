set(GIT_REPO "https://github.com/boostorg/filesystem.git")

function(vad_live)
  message("run VAD_LIVE for boost_filesystem")
  
  git_clone(boost_filesystem)
  
  # FIXME
  #somehow inc dir forwarding is not working for pure imported targets (e.g. config, utility etc...)
  
  set(_FORWARD_DEPS boost_config boost_system boost_type_traits boost_iterator boost_smart_ptr boost_io boost_functional boost_range)

  foreach(DEP ${_FORWARD_DEPS}) 
    get_target_property(_BOOST_FS_EXTRA_INC_SINGLE ${DEP} INTERFACE_INCLUDE_DIRECTORIES)
    list(APPEND _BOOST_FS_EXTRA_INC ${_BOOST_FS_EXTRA_INC_SINGLE})
  endforeach()
  
  include_directories("${_BOOST_FS_EXTRA_INC}")
  include_directories("${VAD_EXTERNAL_ROOT}/boost_filesystem/include")
  
  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_filesystem/src/*")
  
  
  add_library(boost_filesystem ${_SRCS})
  target_link_libraries(boost_filesystem PUBLIC ${_FORWARD_DEPS})
endfunction()
