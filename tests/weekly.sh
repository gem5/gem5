#!/bin/bash

# Copyright (c) 2021 The Regents of the University of California
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

# We assume the lone argument is the number of threads. If no argument is
# given we default to one.
threads=1
if [[ $# -gt 0 ]]; then
    threads=$1
fi

# Run the gem5 very-long tests.
docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}"/tests --rm gcr.io/gem5-test/ubuntu-20.04_all-dependencies \
        ./main.py run --length very-long -j${threads} -t${threads}

# For the GPU tests we compile and run GCN3_X86 inside a gcn-gpu container.
docker pull gcr.io/gem5-test/gcn-gpu:latest
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" gcr.io/gem5-test/gcn-gpu:latest bash -c \
    "scons build/GCN3_X86/gem5.opt -j${threads} \
        || (rm -rf build && scons build/GCN3_X86/gem5.opt -j${threads})"

# before pulling gem5 resources, make sure it doesn't exist already
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
       "${gem5_root}" gcr.io/gem5-test/gcn-gpu:latest bash -c \
       "rm -rf ${gem5_root}/gem5-resources"

# test LULESH
# Pull gem5 resources to the root of the gem5 directory -- currently the
# pre-built binares for LULESH are out-of-date and won't run correctly with
# ROCm 4.0.  In the meantime, we can build the binary as part of this script.
# Moreover, DNNMark builds a library and thus doesn't have a binary, so we
# need to build it before we run it.
git clone -b develop https://gem5.googlesource.com/public/gem5-resources \
    "${gem5_root}/gem5-resources"

mkdir -p tests/testing-results

# build LULESH
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
       "${gem5_root}/gem5-resources/src/gpu/lulesh" \
       -u $UID:$GID gcr.io/gem5-test/gcn-gpu:latest bash -c \
       "make"

# LULESH is heavily used in the HPC community on GPUs, and does a good job of
# stressing several GPU compute and memory components
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" gcr.io/gem5-test/gcn-gpu:latest build/GCN3_X86/gem5.opt \
    configs/example/apu_se.py -n3 --mem-size=8GB \
    --benchmark-root="${gem5_root}/gem5-resources/src/gpu/lulesh/bin" -c lulesh

# test DNNMark
# setup cmake for DNNMark
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
     "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
     gcr.io/gem5-test/gcn-gpu:latest bash -c "./setup.sh HIP"

# make the DNNMark library
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark/build" \
    gcr.io/gem5-test/gcn-gpu:latest bash -c "make -j${threads}"

# generate cachefiles -- since we are testing gfx801 and 4 CUs (default config)
# in tester, we want cachefiles for this setup
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
    "-v${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
    gcr.io/gem5-test/gcn-gpu:latest bash -c \
    "python3 generate_cachefiles.py cachefiles.csv --gfx-version=gfx801 \
    --num-cus=4"

# generate mmap data for DNNMark (makes simulation much faster)
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark" gcr.io/gem5-test/gcn-gpu:latest bash -c \
    "g++ -std=c++0x generate_rand_data.cpp -o generate_rand_data"

docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark" gcr.io/gem5-test/gcn-gpu:latest bash -c \
    "./generate_rand_data"

# now we can run DNNMark!
# DNNMark is representative of several simple (fast) layers within ML
# applications, which are heavily used in modern GPU applications.  So, we want
# to make sure support for these applications are tested.  Run three variants:
# fwd_softmax, bwd_bn, fwd_pool; these tests ensure we run a variety of ML kernels,
# including both inference and training
docker run --rm --volume "${gem5_root}":"${gem5_root}" -v \
       "${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
       -w "${gem5_root}/gem5-resources/src/gpu/DNNMark" gcr.io/gem5-test/gcn-gpu \
       "${gem5_root}/build/GCN3_X86/gem5.opt" "${gem5_root}/configs/example/apu_se.py" -n3 \
       --benchmark-root="${gem5_root}/gem5-resources/src/gpu/DNNMark/build/benchmarks/test_fwd_softmax" \
       -c dnnmark_test_fwd_softmax \
       --options="-config ${gem5_root}/gem5-resources/src/gpu/DNNMark/config_example/softmax_config.dnnmark \
       -mmap ${gem5_root}/gem5-resources/src/gpu/DNNMark/mmap.bin"

docker run --rm --volume "${gem5_root}":"${gem5_root}" -v \
       "${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
       -w "${gem5_root}/gem5-resources/src/gpu/DNNMark" gcr.io/gem5-test/gcn-gpu \
       "${gem5_root}/build/GCN3_X86/gem5.opt" "${gem5_root}/configs/example/apu_se.py" -n3 \
       --benchmark-root="${gem5_root}/gem5-resources/src/gpu/DNNMark/build/benchmarks/test_fwd_pool" \
       -c dnnmark_test_fwd_pool \
       --options="-config ${gem5_root}/gem5-resources/src/gpu/DNNMark/config_example/pool_config.dnnmark \
       -mmap ${gem5_root}/gem5-resources/src/gpu/DNNMark/mmap.bin"

docker run --rm --volume "${gem5_root}":"${gem5_root}" -v \
       "${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
       -w "${gem5_root}/gem5-resources/src/gpu/DNNMark" gcr.io/gem5-test/gcn-gpu \
       "${gem5_root}/build/GCN3_X86/gem5.opt" "${gem5_root}/configs/example/apu_se.py" -n3 \
       --benchmark-root="${gem5_root}/gem5-resources/src/gpu/DNNMark/build/benchmarks/test_bwd_bn" \
       -c dnnmark_test_bwd_bn \
       --options="-config ${gem5_root}/gem5-resources/src/gpu/DNNMark/config_example/bn_config.dnnmark \
       -mmap ${gem5_root}/gem5-resources/src/gpu/DNNMark/mmap.bin"

# Delete the gem5 resources repo we created -- need to do in docker because of
# cachefiles DNNMark creates
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
       "${gem5_root}" gcr.io/gem5-test/gcn-gpu:latest bash -c \
       "rm -rf ${gem5_root}/gem5-resources"
