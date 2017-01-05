set(GIT_REPO "https://github.com/boostorg/filesystem.git")

function(vad_live)
  message("run VAD_LIVE for boost_filesystem")
  
  git_clone(boost_filesystem)

  #get_target_property(_BOOST_FS_EXTRA_INC_SINGLE boost_smart_ptr INTERFACE_INCLUDE_DIRECTORIES)
  #list(APPEND _BOOST_FS_EXTRA_INC ${_BOOST_FS_EXTRA_INC_SINGLE})
  
  #include_directories("${_BOOST_FS_EXTRA_INC}")
  include_directories("${VAD_EXTERNAL_ROOT}/boost_filesystem/include")
  
  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_filesystem/src/*")
  
  
  add_library(boost_filesystem ${_SRCS})
  target_link_libraries(boost_filesystem boost_config boost_system boost_type_traits boost_iterator boost_smart_ptr boost_io boost_functional boost_range)
endfunction()
