# By default, the code is compiled for a "big endian" machine.
# To compile on a "little endian" machine set the LITTLE_ENDIAN flag.
# To make smaller object code, but run a little slower, don't use UNROLL_LOOPS.
# To use NIST's modified SHA of 7/11/94, define USE_MODIFIED_SHA

# CC = gcc
# STRIP_CMD = strip

CC = aarch64-linux-gnu-gcc
STRIP_CMD = aarch64-linux-gnu-strip

# CC = riscv64-linux-gnu-gcc
# STRIP_CMD = riscv64-linux-gnu-strip

CFLAGS = -O3 -Wall -static

sha:	sha_driver.o sha.o
	$(CC) -o $@ sha_driver.o sha.o $(CFLAGS)
	$(STRIP_CMD) $@

clean:
	rm -rf *.o sha output*
