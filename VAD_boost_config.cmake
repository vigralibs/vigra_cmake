set(GIT_REPO "https://github.com/boostorg/config.git")

function(vad_live)
  message("run VAD_LIVE for boost_config")
  
  git_clone(boost_config)
  
  # WARNING very important! disables auto-linking for boost!
  set(_CFG_PATH "${VAD_EXTERNAL_ROOT}/boost_config/include/boost/config/user.hpp")
  file(READ ${_CFG_PATH} FILECONTENT)
  string(REPLACE "// #define BOOST_ALL_NO_LIB" "#define BOOST_ALL_NO_LIB" FILECONTENT ${FILECONTENT})
  file(WRITE ${_CFG_PATH} ${FILECONTENT})

  # TODO BOOST_CONFIG::BOOST_CONFIG ... check conventions with francesco
  add_library(boost_config INTERFACE IMPORTED)
  set_target_properties(boost_config PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${VAD_EXTERNAL_ROOT}/boost_config/include")
endfunction()
