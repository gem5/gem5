ACLOCAL_AMFLAGS=-I m4

SUBDIRS=. tests

MOSTLYCLEANFILES=

lib_LTLIBRARIES = libfputils.la

include_HEADERS = \
	include/fputils/fp80.h \
	include/fputils/fp64.h \
	include/fputils/fptypes.h

libfputils_la_SOURCES = \
	include/fputils/fp80.h \
	include/fputils/fp64.h \
	include/fputils/fptypes.h \
	fpbits.h \
	fp64.c \
	fp80.c


include $(top_srcdir)/doxygen.am

MOSTLYCLEANFILES += $(DX_CLEANFILES)
