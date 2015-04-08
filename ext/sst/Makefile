# These two variables are designed to be modifiable.
SST_VERSION=SST-trunk
GEM5_LIB=gem5_opt

LDFLAGS=-shared -fno-common ${shell pkg-config ${SST_VERSION} --libs} -L../../build/ARM
CXXFLAGS=-std=c++0x -g -O2 -fPIC ${shell pkg-config ${SST_VERSION} --cflags} ${shell python-config --includes} -I../../build/ARM
CPPFLAGS+=-MMD -MP
SRC=$(wildcard *.cc)

.PHONY: clean all

all: libgem5.so

libgem5.so: $(SRC:%.cc=%.o)
	${CXX} ${CPPFLAGS} ${LDFLAGS} $? -o $@ -l${GEM5_LIB}

-include $(SRC:%.cc=%.d)

clean:
	${RM} *.[do] libgem5.so
