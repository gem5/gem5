AC_INIT(libfputils, 1.0, andreas@sandberg.pp.se)

AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([foreign -Wall])

DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)
DX_MAN_FEATURE(OFF)
DX_INIT_DOXYGEN([libfputils])

AC_REQUIRE_AUX_FILE([tap-driver.sh])

AC_PROG_CC
AC_PROG_CC_C99
AC_PROG_LIBTOOL
AC_PROG_AWK

if test "x$ac_cv_prog_cc_c99" = "xno"; then
  AC_MSG_ERROR([Could not enable C99 support in compiler.])
fi

AM_CFLAGS="-Wall -Werror"
AM_CPPFLAGS="-I\$(abs_top_srcdir)/include"

AC_SUBST(AM_CFLAGS)
AC_SUBST(AM_CPPFLAGS)

AC_CONFIG_HEADERS([config.h])

AC_CONFIG_FILES([ \
        Doxyfile \
	Makefile \
        tests/Makefile \
])
AC_OUTPUT
