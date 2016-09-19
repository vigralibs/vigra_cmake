add_subdirectory("${DEPS_EXTERNAL_ROOT}/ZLIB" "${DEPS_EXTERNAL_ROOT}/ZLIB/build_external_dep" EXCLUDE_FROM_ALL)

# We will create a new target ZLIB::ZLIB which links to the target
# zlib from the zlib repo, in order to mimick what is provided
# by the vanilla FindZLIB cmake macro.
if(NOT TARGET ZLIB::ZLIB)
  add_library(ZLIB INTERFACE)
  set_target_properties(ZLIB PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${DEPS_EXTERNAL_ROOT}/ZLIB")
    set_target_properties(ZLIB PROPERTIES INTERFACE_LINK_LIBRARIES zlib)
    add_library(ZLIB::ZLIB ALIAS ZLIB)
endif()
