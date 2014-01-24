build:
	aarch64-linux-gnu-gcc -c  -DPHYS_OFFSET=0x80000000 -DCNTFRQ=0x01800000 -DUART_BASE=0x1c090000 -DSYSREGS_BASE=0x1c010000 -DGIC_DIST_BASE=0x2c001000 -DGIC_CPU_BASE=0x2c002000 -Dkernel=0x80080000 -Dmbox=0x8000fff8 -Ddtb=0x80000100 -o boot_emm.o -march=armv8-a boot.S
	aarch64-linux-gnu-ld -o boot_emm.arm64 -N -Ttext 0x00000010 boot_emm.o -non_shared -static
	rm boot_emm.o