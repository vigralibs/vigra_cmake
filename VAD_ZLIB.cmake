include(VigraAddDep)

set(GIT_REPO "https://github.com/madler/zlib.git")

function(vad_live)
    # Clone and add the subdirectory.
    git_clone(ZLIB)
    add_subdirectory("${VAD_EXTERNAL_ROOT}/ZLIB" "${VAD_EXTERNAL_ROOT}/ZLIB/build_external_dep")

    # We are now going to reconstruct the targets/variables provided by the standard FindZLIB module,
    add_library(_VAD_ZLIB_STUB INTERFACE)
    # The ZLIB library provides two public includes, zconf.h and zlib.h. In the live build
    # they sit in different directories.
    list(APPEND _ZLIB_INCLUDE_DIRS "${VAD_EXTERNAL_ROOT}/ZLIB" "${VAD_EXTERNAL_ROOT}/ZLIB/build_external_dep")
    set_property(TARGET _VAD_ZLIB_STUB PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${_ZLIB_INCLUDE_DIRS})
    if(VAD_PREFER_STATIC OR VAD_ZLIB_PREFER_STATIC)
        set_target_properties(_VAD_ZLIB_STUB PROPERTIES INTERFACE_LINK_LIBRARIES zlibstatic)
    else()
        set_target_properties(_VAD_ZLIB_STUB PROPERTIES INTERFACE_LINK_LIBRARIES zlib)
    endif()
    # ZLIB::ZLIB is the imported target provided by FindZLIB.
    add_library(ZLIB::ZLIB ALIAS _VAD_ZLIB_STUB)
    set(ZLIB_FOUND TRUE CACHE INTERNAL "")

    # Set the global variables, ZLIB_INCLUDE_DIRS and ZLIB_LIBRARIES, provided by FindZLIB.
    message(STATUS "Setting ZLIB_INCLUDE_DIRS to '${_ZLIB_INCLUDE_DIRS}'.")
    set(ZLIB_INCLUDE_DIRS "${_ZLIB_INCLUDE_DIRS}" CACHE STRING "")
    if(VAD_PREFER_STATIC OR VAD_ZLIB_PREFER_STATIC)
        message(STATUS "Setting ZLIB_LIBRARIES to the zlibstatic target from the live dependency.")
        set(ZLIB_LIBRARIES zlibstatic CACHE STRING "")
    else()
        message(STATUS "Setting ZLIB_LIBRARIES to the zlib target from the live dependency.")
        set(ZLIB_LIBRARIES zlib CACHE STRING "")
    endif()
    # NOTE: ZLIB_LIBRARY is not part of the public info exported by FindZLIB, but some packages use it nevertheless.
    set(ZLIB_LIBRARY "${ZLIB_LIBRARIES}" CACHE STRING "")
    # Same for the include dir.
    set(ZLIB_INCLUDE_DIR "${ZLIB_INCLUDE_DIRS}" CACHE STRING "")
    mark_as_advanced(FORCE ZLIB_INCLUDE_DIRS)
    mark_as_advanced(FORCE ZLIB_LIBRARIES)
    mark_as_advanced(FORCE ZLIB_INCLUDE_DIR)
    mark_as_advanced(FORCE ZLIB_LIBRARY)
    # Fetch the full version string from zlib.h, and populate the versioning variable provided by FindZLIB.
    file(READ ${VAD_EXTERNAL_ROOT}/ZLIB/zlib.h _zlib_h_contents)
    string(REGEX REPLACE ".*#define[ \t]+ZLIB_VERSION[ \t]+\"([-0-9A-Za-z.]+)\".*"
        "\\1" ZLIB_FULL_VERSION ${_zlib_h_contents})
    set(ZLIB_VERSION_STRING ${ZLIB_FULL_VERSION} CACHE INTERNAL "")
    string(REGEX MATCHALL "([0-9]+)" _ZLIB_VERSION_LIST ${ZLIB_FULL_VERSION})
    list(LENGTH _ZLIB_VERSION_LIST _ZLIB_VERSION_LIST_LENGTH)
    if(_ZLIB_VERSION_LIST_LENGTH GREATER 0)
        list(GET _ZLIB_VERSION_LIST 0 ZLIB_VERSION_MAJOR)
        set(ZLIB_VERSION_MAJOR ${ZLIB_VERSION_MAJOR} CACHE INTERNAL "")
        set(ZLIB_MAJOR_VERSION ${ZLIB_VERSION_MAJOR} CACHE INTERNAL "")
        message(STATUS "zlib major version: ${ZLIB_VERSION_MAJOR}")
    endif()
    if(_ZLIB_VERSION_LIST_LENGTH GREATER 1)
        list(GET _ZLIB_VERSION_LIST 1 ZLIB_VERSION_MINOR)
        set(ZLIB_VERSION_MINOR ${ZLIB_VERSION_MINOR} CACHE INTERNAL "")
        set(ZLIB_MINOR_VERSION ${ZLIB_VERSION_MINOR} CACHE INTERNAL "")
        message(STATUS "zlib minor version: ${ZLIB_VERSION_MINOR}")
    endif()
    if(_ZLIB_VERSION_LIST_LENGTH GREATER 2)
        list(GET _ZLIB_VERSION_LIST 2 ZLIB_VERSION_PATCH)
        set(ZLIB_VERSION_PATCH ${ZLIB_VERSION_PATCH} CACHE INTERNAL "")
        set(ZLIB_PATCH_VERSION ${ZLIB_VERSION_PATCH} CACHE INTERNAL "")
        message(STATUS "zlib patch version: ${ZLIB_VERSION_PATCH}")
    endif()
    if(_ZLIB_VERSION_LIST_LENGTH GREATER 3)
        list(GET _ZLIB_VERSION_LIST 3 ZLIB_VERSION_TWEAK)
        set(ZLIB_VERSION_TWEAK ${ZLIB_VERSION_TWEAK} CACHE INTERNAL "")
        message(STATUS "zlib tweak version: ${ZLIB_VERSION_TWEAK}")
    endif()

    # The zlib build system does not correctly set the include path for its executables when zlib
    # is built as a sub project. Fix it by adding explicitly the missing path.
    function(_zlib_exec_fix_include_dir TRG)
        if(NOT TARGET "${TRG}")
            return()
        endif()
        get_property(_include_dir TARGET ${TRG} PROPERTY INCLUDE_DIRECTORIES)
        list(APPEND _include_dir "${VAD_EXTERNAL_ROOT}/ZLIB")
        set_property(TARGET ${TRG} PROPERTY INCLUDE_DIRECTORIES ${_include_dir})
    endfunction()
    _zlib_exec_fix_include_dir(minigzip)
    _zlib_exec_fix_include_dir(example)
    _zlib_exec_fix_include_dir(example64)
    _zlib_exec_fix_include_dir(minigzip64)
endfunction()
