set(GIT_REPO "https://github.com/boostorg/filesystem.git")

function(vad_deps)
message("boost fs deps !")
endfunction()

function(vad_live)
  message("run VAD_LIVE for boost_filesystem")
  
  git_clone(boost_filesystem)
  
  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_filesystem/src/*")
  
  set(_BOOST_FS_DEPS boost_config boost_system boost_type_traits boost_iterator boost_smart_ptr boost_io boost_functional boost_range)
  
  add_library(boost_filesystem ${_SRCS})
  target_include_directories(boost_filesystem PUBLIC "${VAD_EXTERNAL_ROOT}/boost_filesystem/include")
  target_link_libraries(boost_filesystem PUBLIC ${_BOOST_FS_DEPS})
  
  add_library(Boost::filesystem ALIAS boost_filesystem)
endfunction()
