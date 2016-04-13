# - Find ALGLIB
# Find the native ALGLIB includes and library
#
#  ALGLIB_INCLUDES    - where to find Alglib includes
#  ALGLIB_LIBRARIES   - List of libraries when using Alglib.
#  ALGLIB_FOUND       - True if Alglib found.

if (ALGLIB_INCLUDE_DIRS)
  # Already in cache, be silent
  set (ALGLIB_FIND_QUIETLY TRUE)
endif (ALGLIB_INCLUDE_DIRS)

find_path (ALGLIB_INCLUDE_DIRS 
    alglibinternal.h
    alglibmisc.h
    ap.h
    dataanalysis.h
    diffequations.h
    fasttransforms.h
    integration.h
    interpolation.h
    linalg.h
    optimization.h
    solvers.h
    specialfunctions.h
    statistics.h
    stdafx.h
    PATHS
    /usr/include/libalglib/
    /usr/local/include/alglib3/
    )

find_library (ALGLIB_LIBRARIES NAMES alglib alglib3)

# handle the QUIETLY and REQUIRED arguments and set ALGLIB_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (ALGLIB DEFAULT_MSG ALGLIB_LIBRARIES ALGLIB_INCLUDE_DIRS)

mark_as_advanced (ALGLIB_LIBRARIES ALGLIB_INCLUDE_DIRS)
