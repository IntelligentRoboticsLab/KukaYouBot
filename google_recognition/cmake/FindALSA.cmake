# Try to find FLAC
# Once done this will define
#  FLAC_FOUND        - Indication as to whether FLAC was found
#  FLAC_INCLUDE_DIRS - The FLAC include directories
#  FLAC_LIBRARIES    - The libraries needed to use LFLACCM

find_package (PkgConfig)
pkg_check_modules (PC_ALSA QUIET libasound)

find_path(ALSA_INCLUDE_DIR NAMES alsa/asoundlib.h
    HINTS @CMAKE_INSTALL_PREFIX@/include ${PC_ALSA_INCLUDEDIR} ${PC_ALSA_INCLUDE_DIRS} /usr/include /usr/local/include /opt/local/include)

find_library (ALSA_LIBRARY NAMES libasound.so ALSA
    HINTS @CMAKE_INSTALL_PREFIX@/lib ${PC_ALSA_LIBDIR} ${PC_ALSA_LIBRARY_DIRS} /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /opt/local/lib)

set (ALSA_LIBRARIES ${ALSA_LIBRARY})
set (ALSA_INCLUDE_DIRS ${ALSA_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args (ALSA DEFAULT_MSG
    ALSA_LIBRARY ALSA_INCLUDE_DIR)

mark_as_advanced (ALSA_INCLUDE_DIR ALSA_LIBRARY)

