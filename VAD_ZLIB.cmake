set(GIT_REPO "https://github.com/madler/zlib.git")

function(vad_system NAME)
    list(REMOVE_ITEM CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
    find_package(ZLIB)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
    add_library(_ZLIB_STUB INTERFACE)
    get_target_property(_ZLIB_STUB_INTERFACE_INCLUDE_DIRECTORIES ZLIB::ZLIB INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(_ZLIB_STUB_IMPORTED_LOCATION ZLIB::ZLIB IMPORTED_LOCATION)
    if(NOT _ZLIB_STUB_IMPORTED_LOCATION)
      get_target_property(_ZLIB_STUB_IMPORTED_LOCATION ZLIB::ZLIB IMPORTED_LOCATION_RELEASE)
    endif()
    if(NOT _ZLIB_STUB_IMPORTED_LOCATION)
      get_target_property(_ZLIB_STUB_IMPORTED_LOCATION ZLIB::ZLIB IMPORTED_LOCATION_DEBUG)
    endif()
    set_target_properties(_ZLIB_STUB PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_ZLIB_STUB_INTERFACE_INCLUDE_DIRECTORIES}")
    target_link_libraries(_ZLIB_STUB INTERFACE "${_ZLIB_STUB_IMPORTED_LOCATION}")
    add_library(ZLIB::ZLIB ALIAS _ZLIB_STUB)
    set(ZLIB_LIBRARIES "${_ZLIB_STUB_IMPORTED_LOCATION}" CACHE FILEPATH "")
    set(ZLIB_INCLUDE_DIRS "${_ZLIB_STUB_INTERFACE_INCLUDE_DIRECTORIES}" CACHE FILEPATH "")
endfunction()

function(vad_live NAME)
    # Clone and add the subdirectory.
    git_clone(${VAD_${NAME}_GIT_REPO} ZLIB)
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
    set(ZLIB_INCLUDE_DIRS "${_ZLIB_INCLUDE_DIRS}" CACHE PATH "")
    if(VAD_PREFER_STATIC OR VAD_ZLIB_PREFER_STATIC)
        message(STATUS "Setting ZLIB_LIBRARIES to the zlibstatic target from the live dependency.")
        set(ZLIB_LIBRARIES zlibstatic CACHE FILEPATH "")
    else()
        message(STATUS "Setting ZLIB_LIBRARIES to the zlib target from the live dependency.")
        set(ZLIB_LIBRARIES zlib CACHE FILEPATH "")
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
