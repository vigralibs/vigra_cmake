set(GIT_REPO "https://github.com/boostorg/filesystem.git")

function(vad_deps)
message("boost fs deps !")
endfunction()

function(vad_live)
  message("run VAD_LIVE for boost_filesystem")
  
  git_clone(boost_filesystem)
    
  include_directories("${VAD_EXTERNAL_ROOT}/boost_filesystem/include")
  
  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_filesystem/src/*")
  
  set(_BOOST_FS_DEPS boost_config boost_system boost_type_traits boost_iterator boost_smart_ptr boost_io boost_functional boost_range)
  
  add_library(boost_filesystem ${_SRCS})
  target_link_libraries(boost_filesystem PUBLIC ${_BOOST_FS_DEPS})
  
  # FIXME
  #somehow inc dir forwarding is not working for pure imported targets (e.g. config, utility etc...)
  forward_target_includes(boost_filesystem ${_BOOST_FS_DEPS})
endfunction()
