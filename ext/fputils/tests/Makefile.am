
TEST_LOG_DRIVER = env AM_TAP_AWK='$(AWK)' $(SHELL) \
                       $(top_srcdir)/tap-driver.sh

LIBS=-lm $(top_builddir)/.libs/libfputils.la libtest.a

FP80_TESTS=fp80_cvtd fp80_cvfd fp80_cvtf
TESTS=$(FP80_TESTS)

check_LIBRARIES=libtest.a
check_PROGRAMS=$(FP80_TESTS)

libtest_a_SOURCES=test_helper.c test_helper.h
