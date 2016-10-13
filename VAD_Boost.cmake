include(VigraAddDep)

function(vad_system)
  # Disable use of boost-cmake: the upstream project has been dead for a while, and
  # the way it is tested for in FindBoost.cmake wreaks havoc due to the fact that
  # we cache the value of Boost_FOUND.
  message(STATUS "Disabling boost-cmake.")
  set(Boost_NO_BOOST_CMAKE TRUE)
  vad_system_default(${ARGN})
endfunction()
