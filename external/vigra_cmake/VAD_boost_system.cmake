set(GIT_REPO "https://github.com/boostorg/system.git")

function(vad_live)
  message("run VAD_LIVE for boost_system")
  
  git_clone(boost_system)

  include_directories("${VAD_EXTERNAL_ROOT}/boost_system/include")
  
  file(GLOB _SRCS "${VAD_EXTERNAL_ROOT}/boost_system/src/*")
  
  add_library(boost_system ${_SRCS})
  target_link_libraries(boost_system boost_config boost_predef boost_assert boost_core)
endfunction()
