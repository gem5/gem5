# Makefile for building various ISA-specific m5_exit tests using Docker

# Path to the gem5 directory root
GEM5_DIR = $(abspath ../../../..)

# Path to libm5.a for each ISA
X86_LIBM5 = "${GEM5_DIR}/util/m5/build/x86/out/m5"
ARM_LIBM5 = "${GEM5_DIR}/util/m5/build/arm64/out/m5"
RISCV_LIBM5 = "${GEM5_DIR}/util/m5/build/riscv/out/m5"

# Docker image to use
DOCKER_IMAGE = ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest

# Targets
TARGETS = bin/x86-addr-m5-exit bin/x86-inst-m5-exit bin/arm-addr-m5-exit bin/arm-inst-m5-exit bin/riscv-inst-m5-exit

# Default target
all: ${TARGETS}

# Rules for x86
bin/x86-addr-m5-exit: bin ${X86_LIBM5}
	docker run --rm  -it --platform linux/amd64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/tests/gem5/m5_util/m5_exit ${DOCKER_IMAGE} gcc m5_exit_addr.c -L /gem5/util/m5/build/x86/out -l m5 -I /gem5/include --static -o bin/x86-addr-m5-exit

bin/x86-inst-m5-exit: bin ${X86_LIBM5}
	docker run --rm -it --platform linux/amd64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/tests/gem5/m5_util/m5_exit ${DOCKER_IMAGE} gcc m5_exit.c -L /gem5/util/m5/build/x86/out -l m5 -I /gem5/include --static -o bin/x86-inst-m5-exit

${X86_LIBM5}:
	docker run --rm -it --platform linux/amd64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/util/m5 ${DOCKER_IMAGE} scons build/x86/out/m5 -j `nproc`

# Rules for ARM
bin/arm-addr-m5-exit: bin ${ARM_LIBM5}
	docker run --rm  -it --platform linux/arm64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/tests/gem5/m5_util/m5_exit ${DOCKER_IMAGE} gcc m5_exit_addr.c -L /gem5/util/m5/build/arm64/out -l m5 -I /gem5/include --static -o bin/arm-addr-m5-exit

bin/arm-inst-m5-exit: bin ${ARM_LIBM5}
	docker run --rm -it --platform linux/arm64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/tests/gem5/m5_util/m5_exit ${DOCKER_IMAGE} gcc m5_exit.c -L /gem5/util/m5/build/arm64/out -l m5 -I /gem5/include --static -o bin/arm-inst-m5-exit

${ARM_LIBM5}:
	docker run --rm -it --platform linux/amd64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/util/m5 ${DOCKER_IMAGE} bash -c 'apt update -y && apt install -y g++-aarch64* && scons build/arm64/out/m5 -j `nproc`'

# Rules for RISCV
bin/riscv-inst-m5-exit: bin ${RISCV_LIBM5}
	docker run --rm -it --platform linux/riscv64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/tests/gem5/m5_util/m5_exit ${DOCKER_IMAGE} gcc m5_exit.c -L /gem5/util/m5/build/riscv/out -l m5 -I /gem5/include --static -o bin/riscv-inst-m5-exit

${RISCV_LIBM5}:
	docker run --rm -it --platform linux/amd64 -u ${UID}:${GID} -v "${GEM5_DIR}":/gem5 -w /gem5/util/m5 ${DOCKER_IMAGE} bash -c 'apt update -y && apt install -y g++-riscv64-* && scons riscv.CROSS_COMPILE=riscv64-linux-gnu- build/riscv/out/m5 -j `nproc`'

# Create bin directory
bin:
	mkdir -p bin

# Clean target
clean:
	- rm -r bin ${GEM5_DIR}/util/m5/build

.PHONY: all clean
