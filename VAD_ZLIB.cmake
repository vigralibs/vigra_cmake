set(GIT_REPO "https://github.com/madler/zlib.git")

function(vad_system NAME)
    # Call the CMake-provided FindZLIB module. We need to remove the current path from the CMake module
    # path so we don't end up calling our FindZLIB override.
    list(REMOVE_ITEM CMAKE_MODULE_PATH "${VAD_CMAKE_ROOT}")
    find_package(ZLIB)
    list(APPEND CMAKE_MODULE_PATH "${VAD_CMAKE_ROOT}")

    if(NOT ZLIB_FOUND)
        set(VAD_ZLIB_SYSTEM_NOT_FOUND TRUE CACHE INTERNAL "")
        return()
    endif()

    # There are some problems here, which we we will solve with some hackery:
    # - CMake's FindZLIB provides a ZLIB::ZLIB *imported* target. Imported target have special scoping
    #   rules different from normal targets: they are visible only to subdirs/subprojects which are children
    #   of the project from which the imported target was defined. We will need to reconstruct and override the
    #   ZLIB::ZLIB target (this is apparently allowed via an alias);
    # - the variables defined by CMake's FindZLIB module have a similar problem, in the sense that they are visible
    #   only to subdirs/subprojects. We will need to make them global cache variables.
    #
    # The technique we will be using is the following: we introduce a new interface target _VAD_ZLIB_STUB, to which we
    # attach ZLIB's properties. Then we introduce an alias for it called ZLIB::ZLIB.
    add_library(_VAD_ZLIB_STUB INTERFACE)
    message(STATUS "zlib include dirs: ${ZLIB_INCLUDE_DIRS}")
    message(STATUS "zlib libraries: ${ZLIB_LIBRARIES}")
    set_target_properties(_VAD_ZLIB_STUB PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ZLIB_INCLUDE_DIRS}")
    target_link_libraries(_VAD_ZLIB_STUB INTERFACE "${ZLIB_LIBRARIES}")
    add_library(ZLIB::ZLIB ALIAS _VAD_ZLIB_STUB)

    # Now store the variables provided by FindZLIB as global cached variable.
    set(ZLIB_INCLUDE_DIRS "${ZLIB_INCLUDE_DIRS}" CACHE STRING "")
    set(ZLIB_LIBRARIES "${ZLIB_LIBRARIES}" CACHE STRING "")
    set(ZLIB_FOUND TRUE CACHE INTERNAL "")
    set(ZLIB_VERSION_STRING ${ZLIB_VERSION_STRING} CACHE INTERNAL "")
    set(ZLIB_VERSION_MAJOR ${ZLIB_VERSION_MAJOR} CACHE INTERNAL "")
    set(ZLIB_VERSION_MINOR ${ZLIB_VERSION_MINOR} CACHE INTERNAL "")
    set(ZLIB_VERSION_PATCH ${ZLIB_VERSION_PATCH} CACHE INTERNAL "")
    set(ZLIB_VERSION_TWEAK ${ZLIB_VERSION_TWEAK} CACHE INTERNAL "")
    set(ZLIB_MAJOR_VERSION ${ZLIB_MAJOR_VERSION} CACHE INTERNAL "")
    set(ZLIB_MINOR_VERSION ${ZLIB_MINOR_VERSION} CACHE INTERNAL "")
    set(ZLIB_PATCH_VERSION ${ZLIB_PATCH_VERSION} CACHE INTERNAL "")
    mark_as_advanced(FORCE ZLIB_INCLUDE_DIRS)
    mark_as_advanced(FORCE ZLIB_LIBRARIES)
endfunction()

function(vad_live NAME)
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
    mark_as_advanced(FORCE ZLIB_INCLUDE_DIRS)
    mark_as_advanced(FORCE ZLIB_LIBRARIES)

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
        get_property(_include_dir TARGET ${TRG} PROPERTY INCLUDE_DIRECTORIES)
        list(APPEND _include_dir "${VAD_EXTERNAL_ROOT}/ZLIB")
        set_property(TARGET ${TRG} PROPERTY INCLUDE_DIRECTORIES ${_include_dir})
    endfunction()
    _zlib_exec_fix_include_dir(minigzip)
    _zlib_exec_fix_include_dir(example)
    _zlib_exec_fix_include_dir(example64)
    _zlib_exec_fix_include_dir(minigzip64)
endfunction()
