ACLOCAL_AMFLAGS=-I m4

SUBDIRS=. tests

MOSTLYCLEANFILES=

lib_LTLIBRARIES = libfputils.la

include_HEADERS = include/fputils/fp80.h

libfputils_la_SOURCES = \
	include/fputils/fp80.h \
	fpbits.h \
	fp80.c


include $(top_srcdir)/doxygen.am

MOSTLYCLEANFILES += $(DX_CLEANFILES)
