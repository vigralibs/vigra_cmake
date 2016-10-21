set(GIT_REPO "https://github.com/glennrp/libpng.git")

function(vad_live)
  git_clone(PNG)
  add_subdirectory("${VAD_EXTERNAL_ROOT}/PNG" "${VAD_EXTERNAL_ROOT}/PNG/build_external_dep")

  # We are now going to reconstruct the targets/variables provided by the standard FindPNG module,
  add_library(_VAD_PNG_STUB INTERFACE)
  # TODO shared/static?
  list(APPEND _PNG_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/PNG" "${VAD_EXTERNAL_ROOT}/PNG/build_external_dep")
  set_property(TARGET _VAD_PNG_STUB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${_PNG_INCLUDE_DIRS})
  if(VAD_PREFER_STATIC OR VAD_PNG_PREFER_STATIC)
      set_target_properties(_VAD_PNG_STUB PROPERTIES INTERFACE_LINK_LIBRARIES pngstatic)
  else()
      set_target_properties(_VAD_PNG_STUB PROPERTIES INTERFACE_LINK_LIBRARIES png)
  endif()
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
