include(VigraAddDep)

set(GIT_REPO "https://github.com/glennrp/libpng.git")

function(vad_live)
  # Clone and add the subdirectory.
  git_clone(PNG)
  set(SKIP_INSTALL_EXPORT ON CACHE BOOL "")
  add_subdirectory("${VAD_EXTERNAL_ROOT}/PNG" "${VAD_EXTERNAL_ROOT}/PNG/build_external_dep")
  unset(SKIP_INSTALL_EXPORT CACHE)

  # We are now going to reconstruct the targets/variables provided by the standard FindPNG module,
  add_library(_VAD_PNG_STUB INTERFACE)
  # Include dirs.
  list(APPEND _PNG_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/PNG" "${VAD_EXTERNAL_ROOT}/PNG/build_external_dep")
  set_property(TARGET _VAD_PNG_STUB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${_PNG_INCLUDE_DIRS})
  # By default PNG is build in both static and shared variants. So, we associate to our PNG target
  # the preferred library type.
  if(VAD_PREFER_STATIC OR VAD_PNG_PREFER_STATIC)
      set_target_properties(_VAD_PNG_STUB PROPERTIES INTERFACE_LINK_LIBRARIES pngstatic)
  else()
      set_target_properties(_VAD_PNG_STUB PROPERTIES INTERFACE_LINK_LIBRARIES png)
  endif()
  # According to FindPNG.cmake, the only case in which we need special definitions is when compiling
  # the static library in cygwin.
  if(CYGWIN)
    if(VAD_PREFER_STATIC OR VAD_PNG_PREFER_STATIC)
      set(PNG_DEFINITIONS PNG_STATIC CACHE INTERNAL)
    endif()
  endif()
  set_target_properties(_VAD_PNG_STUB PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${PNG_DEFINITIONS}")
  # PNG has a mandatory dependency on ZLIB.
  target_link_libraries(_VAD_PNG_STUB INTERFACE ZLIB::ZLIB)
  add_library(PNG::PNG ALIAS _VAD_PNG_STUB)
  set(PNG_FOUND TRUE CACHE INTERNAL "")

  # Setup the global variables.
  message(STATUS "Setting PNG_INCLUDE_DIRS to '${_PNG_INCLUDE_DIRS}'.")
  set(PNG_INCLUDE_DIRS "${_PNG_INCLUDE_DIRS}" CACHE STRING "")
  if(VAD_PREFER_STATIC OR VAD_PNG_PREFER_STATIC)
      message(STATUS "Setting PNG_LIBRARIES to the pngstatic target from the live dependency.")
      set(PNG_LIBRARIES pngstatic CACHE STRING "")
  else()
      message(STATUS "Setting PNG_LIBRARIES to the png target from the live dependency.")
      set(PNG_LIBRARIES png CACHE STRING "")
  endif()
  mark_as_advanced(FORCE PNG_INCLUDE_DIRS)
  mark_as_advanced(FORCE PNG_LIBRARIES)
  # TODO definitions.
  # Version string.
  file(STRINGS "${VAD_EXTERNAL_ROOT}/PNG/png.h" png_version_str REGEX "^#define[ \t]+PNG_LIBPNG_VER_STRING[ \t]+\".+\"")
  string(REGEX REPLACE "^#define[ \t]+PNG_LIBPNG_VER_STRING[ \t]+\"([^\"]+)\".*" "\\1" PNG_VERSION_STRING "${png_version_str}")
  set(PNG_VERSION_STRING ${PNG_VERSION_STRING} CACHE INTERNAL "")
endfunction()
