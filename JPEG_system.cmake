find_package(JPEG REQUIRED)

# The vanilla CMake macro does not provide (yet) an imported target,
# so we built it here.
if(NOT TARGET JPEG::JPEG)
    add_library(JPEG::JPEG UNKNOWN IMPORTED)
    set_target_properties(JPEG::JPEG PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${JPEG_INCLUDE_DIR}")
    set_target_properties(JPEG::JPEG PROPERTIES
      IMPORTED_LOCATION "${JPEG_LIBRARIES}")
endif()
