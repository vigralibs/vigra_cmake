set(GIT_REPO "https://github.com/boostorg/boost.git")

function(vad_system)
# TODO args might change!
if (NOT _vad_boost_searching)
  message("rec var: ${_vad_boost_searching}")
  set(_vad_boost_searching true)
  vad_system_default(${ARGN} COMPONENTS filesystem system)
  set(_vad_boost_searching)
endif()
endfunction()

function(vad_live)
  message("run VAD_LIVE for Boost")
  
  # WARNING very important! disables auto-linking for boost!
  set(_CFG_PATH "${VAD_EXTERNAL_ROOT}/boost_config/include/boost/config/user.hpp")
  file(READ ${_CFG_PATH} FILECONTENT)
  string(REPLACE "// #define BOOST_ALL_NO_LIB" "#define BOOST_ALL_NO_LIB" FILECONTENT ${FILECONTENT})
  file(WRITE ${_CFG_PATH} ${FILECONTENT})
  
  
  # required for (all?) others
  vigra_add_dep(boost_config REQUIRED LIVE)
  
  #required for system...
  vigra_add_dep(boost_predef REQUIRED LIVE)
  vigra_add_dep(boost_assert REQUIRED LIVE)
  vigra_add_dep(boost_core REQUIRED LIVE)
  
  #required by mpl
  vigra_add_dep(boost_preprocessor REQUIRED LIVE)
  
  #required by iterator
  vigra_add_dep(boost_mpl REQUIRED LIVE)
  vigra_add_dep(boost_static_assert REQUIRED LIVE)
  vigra_add_dep(boost_detail REQUIRED LIVE)
  
  #required for smart_ptr
  vigra_add_dep(boost_throw_exception REQUIRED LIVE)
  
  # required for boost_filesystem
  vigra_add_dep(boost_system REQUIRED LIVE)
  vigra_add_dep(boost_type_traits REQUIRED LIVE)
  vigra_add_dep(boost_iterator REQUIRED LIVE)
  vigra_add_dep(boost_smart_ptr REQUIRED LIVE)
  vigra_add_dep(boost_io REQUIRED LIVE)
  vigra_add_dep(boost_functional REQUIRED LIVE)
  vigra_add_dep(boost_range REQUIRED LIVE)
  #vigra_add_dep(boost_utility REQUIRED LIVE)
  
  # TODO handle with components...
  vigra_add_dep(boost_filesystem REQUIRED LIVE)
  
  set(BOOST_FOUND TRUE CACHE INTERNAL "")
endfunction()
