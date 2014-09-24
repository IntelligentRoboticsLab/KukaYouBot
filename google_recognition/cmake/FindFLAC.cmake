# Try to find FLAC
# Once done this will define
#  FLAC_FOUND        - Indication as to whether FLAC was found
#  FLAC_INCLUDE_DIRS - The FLAC include directories
#  FLAC_LIBRARIES    - The libraries needed to use LFLACCM

find_package (PkgConfig)
pkg_check_modules (PC_FLAC QUIET libFLAC)

find_path(FLAC_INCLUDE_DIR NAMES FLAC/all.h
    HINTS @CMAKE_INSTALL_PREFIX@/include ${PC_FLAC_INCLUDEDIR} ${PC_FLAC_INCLUDE_DIRS} /usr/include /usr/local/include /opt/local/include)

find_library (FLAC_LIBRARY NAMES libFLAC FLAC
    HINTS @CMAKE_INSTALL_PREFIX@/lib ${PC_FLAC_LIBDIR} ${PC_FLAC_LIBRARY_DIRS} /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /opt/local/lib)

set (FLAC_LIBRARIES ${FLAC_LIBRARY})
set (FLAC_INCLUDE_DIRS ${FLAC_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args (FLAC DEFAULT_MSG
    FLAC_LIBRARY FLAC_INCLUDE_DIR)

mark_as_advanced (FLAC_INCLUDE_DIR FLAC_LIBRARY)
