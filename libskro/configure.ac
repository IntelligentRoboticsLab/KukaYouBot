#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT(libskro, 0.1.0, stephan@synkhronix.com)
AC_CONFIG_SRCDIR([src/skro.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CC
LT_INIT
AM_INIT_AUTOMAKE([1.9 foreign])

# Checks for libraries.

# Checks for header files.
AC_CHECK_HEADERS([stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_CHECK_LIB([soem], [ecx_init])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
