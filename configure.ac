#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([FULL-PACKAGE-NAME], [VERSION], [BUG-REPORT-ADDRESS])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([1.10 foreign dist-bzip2])

# Checks for programs.
AC_PROG_CC

# Checks for libraries.

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/time.h unistd.h])

# dnl our libraries and install dirs use GST_API_VERSION in the filename
# dnl to allow side-by-side installation of different API versions
GST_API_VERSION=1.0
AC_SUBST(GST_API_VERSION)
AC_DEFINE_UNQUOTED(GST_API_VERSION, "$GST_API_VERSION", [GStreamer API Version])

GST_PLUGIN_PATH=gstreamer-${GST_API_VERSION}
AC_SUBST(GST_PLUGIN_PATH)
AC_DEFINE_UNQUOTED(GST_PLUGIN_PATH, "$GST_PLUGIN_PATH", [GStreamer PLUGIN Path])

PKG_PROG_PKG_CONFIG
PKG_CHECK_MODULES([GLIB], [glib-2.0])

# checks for Argument Enable

AC_ARG_WITH(
		[extrapath],
		AS_HELP_STRING([--with-extrapath],[with a extra dependency path for gstreamer]),
		[EXTRAPATH=$withval],
		[EXTRAPATH=no]
)
AC_SUBST([EXTRAPATH])

AC_ARG_WITH(
		[extrapath_lib],
		AS_HELP_STRING([--with-extrapath_lib],[with a extra dependency lib path for gstreamer]),
		[EXTRAPATH_LIB=$withval],
		[EXTRAPATH_LIB=no]
)
AC_SUBST([EXTRAPATH_LIB])

AC_ARG_WITH(
		[extrapath_include],
		AS_HELP_STRING([--with-extrapath_include],[with a extra dependency include path for gstreamer]),
		[EXTRAPATH_INCLUDE=$withval],
		[EXTRAPATH_INCLUDE=no]
)
AC_SUBST([EXTRAPATH_INCLUDE])

# Initialize libtool
LT_PREREQ([2.2])
LT_INIT([disable-static])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL

# Checks for library functions.
# disable because of runtime error 'undefined function : rpl_malloc'
#AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
