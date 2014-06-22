# Process this file with autoconf to produce a configure script

AC_PREREQ(2.57)
AC_INIT([xf86-input-onetouch],
	1.3.0)

AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_AUX_DIR(.)
AM_INIT_AUTOMAKE([dist-bzip2])

AM_MAINTAINER_MODE

# Require xorg-macros: XORG_CWARNFLAGS, XORG_CHANGELOG
m4_ifndef([XORG_MACROS_VERSION], [AC_FATAL([must install xorg-macros 1.2 or later before running autoconf/autogen])])
XORG_MACROS_VERSION(1.2)
AM_CONFIG_HEADER([config.h])

# Checks for programs.
AC_DISABLE_STATIC
AC_PROG_LIBTOOL
AC_PROG_CC
XORG_CWARNFLAGS

AH_TOP([#include "xorg-server.h"])

AC_ARG_WITH(xorg-module-dir,
            AC_HELP_STRING([--with-xorg-module-dir=DIR],
                           [Default xorg module directory [[default=$libdir/xorg/modules]]]),
            [moduledir="$withval"],
            [moduledir="$libdir/xorg/modules"])
inputdir=${moduledir}/input
AC_SUBST(inputdir)

AC_PATH_X

# Checks for extensions
XORG_DRIVER_CHECK_EXT(XINPUT, inputproto)

# Checks for pkg-config packages
PKG_CHECK_MODULES(XORG, [xorg-server >= 1.10.0] xproto $REQUIRED_MODULES)
sdkdir=$(pkg-config --variable=sdkdir xorg-server)
XORG_CFLAGS="$CWARNFLAGS $XORG_CFLAGS"

# Checks for libraries.

# Checks for header files.
AC_HEADER_STDC

DRIVER_NAME=onetouch
AC_SUBST([DRIVER_NAME])

XORG_MANPAGE_SECTIONS
XORG_RELEASE_VERSION
XORG_CHANGELOG

AC_OUTPUT([Makefile src/Makefile tools/Makefile])

