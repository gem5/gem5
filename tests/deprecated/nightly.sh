#!/bin/bash

# Copyright (c) 2021-2023 The Regents of the University of California
# All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set -e
set -x

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
gem5_root="${dir}/.."

# The per-container Docker memory limit.
docker_mem_limit="18g"

# The docker tag to use (varies between develop, and versions on the staging
# branch)
tag="latest"

# The first argument is the number of threads to be used for compilation. If no
# argument is given we default to one.
compile_threads=1
if [[ $# -gt 0 ]]; then
    compile_threads=$1
fi

# The second argument is the number of threads used to run gem5 (one gem5
# instance per thread). If no argument is given we default to one.
run_threads=1
if [[ $# -gt 1 ]]; then
    run_threads=$2
fi

# The third argument is the GPU ISA to run. If no argument is given we default
# to GCN3_X86.
gpu_isa=GCN3_X86
if [[ $# -gt 2 ]]; then
    gpu_isa=$3
fi

if [[ "$gpu_isa" != "GCN3_X86" ]] && [[ "$gpu_isa" != "VEGA_X86" ]]; then
    echo "Invalid GPU ISA: $gpu_isa"
    exit 1
fi

unit_test () {
    build=$1

    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --memory="${docker_mem_limit}" --rm \
        ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
            scons build/ALL/unittests.${build} -j${compile_threads} \
            --ignore-style
}

# Ensure we have the latest docker images.
docker pull ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag}

# Run the unit tests.
unit_test opt
unit_test debug

# Run the gem5 long tests.
docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}"/tests --memory="${docker_mem_limit}" --rm \
    ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
        ./main.py run --length long -j${compile_threads} -t${run_threads} -vv

# Unfortunately, due docker being unable run KVM, we do so separately.
# This script excluses all tags, includes all tests tagged as "kvm", then
# removes all those part of the 'very-long' (weekly) tests, or for compilation
# to '.debug' or '.fast'. We also remove ARM targets as our Jenkins is an X86
# system. Users wishing to run this script elsewhere should be aware of this.
# Note: we delete the build directory here. It was build in the
# "ubuntu-22.04_all-dependencies" docker image which may not be compatible with
# the host environment.
rm -rf "${gem5_root}/build"
cd "${gem5_root}/tests"
./main.py run -j${compile_threads} -vv \
    --exclude-tags ".*" --include-tags kvm --exclude-tags very\-long \
    --exclude-tags debug --exclude-tags fast --exclude-tags ARM \
    --exclude-tags aarch64
cd "${gem5_root}"

# For the GPU tests we compile and run the GPU ISA inside a gcn-gpu container.
docker pull ghcr.io/gem5/gcn-gpu:${tag}
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" \
    ghcr.io/gem5/gcn-gpu:${tag}  bash -c \
    "scons build/${gpu_isa}/gem5.opt -j${compile_threads} --ignore-style \
        || (rm -rf build && scons build/${gpu_isa}/gem5.opt \
        -j${compile_threads} --ignore-style)"

# get square
wget -qN http://dist.gem5.org/dist/develop/test-progs/square/square

mkdir -p tests/testing-results

# Square is the simplest, fastest, more heavily tested GPU application
# Thus, we always want to run this in the nightly regressions to make sure
# basic GPU functionality is working.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" \
    ghcr.io/gem5/gcn-gpu:${tag}  build/${gpu_isa}/gem5.opt \
    configs/example/apu_se.py --reg-alloc-policy=dynamic -n3 -c square

# get HeteroSync
wget -qN http://dist.gem5.org/dist/develop/test-progs/heterosync/gcn3/allSyncPrims-1kernel

# run HeteroSync sleepMutex -- 16 WGs (4 per CU in default config), each doing
# 10 Ld/St per thread and 4 iterations of the critical section is a reasonable
# moderate contention case for the default 4 CU GPU config and help ensure GPU
# atomics are tested.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}"  --memory="${docker_mem_limit}" \
    ghcr.io/gem5/gcn-gpu:${tag} build/${gpu_isa}/gem5.opt \
    configs/example/apu_se.py --reg-alloc-policy=dynamic -n3 -c \
    allSyncPrims-1kernel --options="sleepMutex 10 16 4"

# run HeteroSync LFBarr -- similar setup to sleepMutex above -- 16 WGs
# accessing unique data and then joining a lock-free barrier, 10 Ld/St per
# thread, 4 iterations of critical section.  Again this is representative of a
# moderate contention case for the default 4 CU GPU config and help ensure GPU
# atomics are tested.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}"  --memory="${docker_mem_limit}" \
    ghcr.io/gem5/gcn-gpu:${tag}  build/${gpu_isa}/gem5.opt \
    configs/example/apu_se.py --reg-alloc-policy=dynamic -n3 -c \
    allSyncPrims-1kernel --options="lfTreeBarrUniq 10 16 4"

# Run an SST test.
build_and_run_SST () {
    isa=$1
    variant=$2

    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --rm  --memory="${docker_mem_limit}" \
        ghcr.io/gem5/sst-env:${tag} bash -c "\
scons build/${isa}/libgem5_${variant}.so -j${compile_threads} \
--without-tcmalloc --duplicate-sources --ignore-style && \
cd ext/sst && \
mv Makefile.linux Makefile && \
make clean; make -j ${compile_threads} && \
sst --add-lib-path=./ sst/example.py && \
cd -;
"
}
build_and_run_SST RISCV opt

build_and_run_systemc () {
    rm -rf "${gem5_root}/build/ARM"
    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --memory="${docker_mem_limit}" --rm \
        ghcr.io/gem5/ubuntu-22.04_min-dependencies:${tag} bash -c "\
scons -j${compile_threads} --ignore-style --duplicate-sources \
build/ARM/gem5.opt && \
scons --with-cxx-config --without-python --without-tcmalloc \
--duplicate-sources USE_SYSTEMC=0  \
-j${compile_threads} build/ARM/libgem5_opt.so \
"

    docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
        "${gem5_root}" --memory="${docker_mem_limit}" --rm \
        ghcr.io/gem5/systemc-env:${tag} bash -c "\
cd util/systemc/gem5_within_systemc && \
make -j${compile_threads} && \
../../../build/ARM/gem5.opt ../../../configs/deprecated/example/se.py -c \
    ../../../tests/test-progs/hello/bin/arm/linux/hello && \
LD_LIBRARY_PATH=../../../build/ARM/:/opt/systemc/lib-linux64/ \
    ./gem5.opt.sc m5out/config.ini && \
cd -; \
"
}
build_and_run_systemc
