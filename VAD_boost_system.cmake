set(GIT_REPO "https://github.com/boostorg/system.git")

function(vad_live)
  message("run VAD_LIVE for boost_system")
  
  git_clone(boost_system)

  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_system/src/*")
  
  add_library(boost_system ${_SRCS})
  target_include_directories(boost_system PUBLIC "${VAD_EXTERNAL_ROOT}/boost_system/include")
  target_link_libraries(boost_system PUBLIC boost_config boost_predef boost_assert boost_core)
  
  add_library(Boost::system ALIAS boost_system)
endfunction()
