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

# The per-container Docker memory limit.
docker_mem_limit="24g"

# The docker tag to use (varies between develop, and versions on the staging
# branch)
tag="latest"

# We assume the first three arguments are the number of threads to use for
# compilation followed by the GPU ISA to test, and finally, the number of
# "run threads", the maximum number of tests to be run at once. By default the
# number of compile threads 1 and the GPU ISA is VEGA_X86. The number of
# "run threads" is equal to the number of compile threads by default.
threads=1
gpu_isa=VEGA_X86
run_threads=1
if [[ $# -eq 1 ]]; then
    threads=$1
    run_threads=${threads}
elif [[ $# -eq 2 ]]; then
    threads=$1
    gpu_isa=$2
elif [[ $# -eq 3 ]]; then
    threads=$1
    gpu_isa=$2
    run_threads=$3
else
    if [[ $# -gt 0 ]]; then
        echo "Invalid number of arguments: $#"
        exit 1
    fi
fi

if [[ "$gpu_isa" != "VEGA_X86" ]]; then
    echo "Invalid GPU ISA: $gpu_isa"
    exit 1
fi

# Run the gem5 very-long tests.
docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}"/tests --memory="${docker_mem_limit}" --rm \
    ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
        ./main.py run --length very-long -j${threads} -t${run_threads} -vv

mkdir -p tests/testing-results

# GPU weekly tests start here
# before pulling gem5 resources, make sure it doesn't exist already
docker run -u $UID:$GID --rm --volume "${gem5_root}":"${gem5_root}" -w \
       "${gem5_root}" --memory="${docker_mem_limit}" \
       ghcr.io/gem5/gcn-gpu:${tag} bash -c \
       "rm -rf ${gem5_root}/gem5-resources"

# delete m5out, Pannotia datasets, and output files in case a failed regression
# run left them around
rm -rf ${gem5_root}/m5out coAuthorsDBLP.graph 1k_128k.gr result.out

# Pull gem5 resources to the root of the gem5 directory -- currently the
# pre-built binares for LULESH are out-of-date and won't run correctly with
# ROCm 4.0.  In the meantime, we can build the binary as part of this script.
# Moreover, DNNMark builds a library and thus doesn't have a binary, so we
# need to build it before we run it.
# Need to pull this first because HACC's docker requires this path to exist
git clone https://github.com/gem5/gem5-resources \
    "${gem5_root}/gem5-resources"


# The following script is to ensure these tests are runnable as the resources
# directory changes over time. The gem5 resources repository stable branch is
# tagged upon the new release for that of the previous release. For example,
# when v22.0 is released, the stable branch will be tagged with "v21.2.X.X"
# prior to the merging of the develop/staging branch into the stable branch.
# This is so a user may revert the gem5-resources sources back to a state
# compatable with a particular major release.
#
# To ensure the v21.2 version of these tests continues to run as future
# versions are released, we run this check. If there's been another release,
# we checkout the correct version of gem5 resources.
#
# Note: We disable this code on the develop branch and just checkout develop.

cd "${gem5_root}/gem5-resources"
git checkout develop
#version_tag=$(git tag | grep "v21.2")
#
#if [[ ${version_tag} != "" ]]; then
#       git checkout "${version_tag}"
#fi
#
cd "${gem5_root}"

# For the GPU tests we compile and run the GPU ISA inside a gcn-gpu container.
# HACC requires setting numerous environment variables to run correctly.  To
# avoid needing to set all of these, we instead build a docker for it, which
# has all these variables pre-set in its Dockerfile
# To avoid compiling gem5 multiple times, all GPU benchmarks will use this
docker pull ghcr.io/gem5/gcn-gpu:${tag}
docker build -t hacc-test-weekly ${gem5_root}/gem5-resources/src/gpu/halo-finder

docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
    "scons build/${gpu_isa}/gem5.opt -j${threads} --ignore-style \
        || rm -rf build && scons build/${gpu_isa}/gem5.opt -j${threads} \
        --ignore-style"

# Some of the apps we test use m5ops (and x86), so compile them for x86
# Note: setting TERM in the environment is necessary as scons fails for m5ops if
# it is not set.
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/util/m5" --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
    "export TERM=xterm-256color ; scons build/x86/out/m5"

# test LULESH
# build LULESH
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
       "${gem5_root}/gem5-resources/src/gpu/lulesh" \
       -u $UID:$GID --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "make"

# LULESH is heavily used in the HPC community on GPUs, and does a good job of
# stressing several GPU compute and memory components
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" \
    hacc-test-weekly build/${gpu_isa}/gem5.opt configs/example/apu_se.py -n3 \
    --mem-size=8GB --reg-alloc-policy=dynamic \
    --benchmark-root="${gem5_root}/gem5-resources/src/gpu/lulesh/bin" -c lulesh

# test DNNMark
# setup cmake for DNNMark
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
     "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
     --memory="${docker_mem_limit}" hacc-test-weekly bash -c "./setup.sh HIP"

# make the DNNMark library
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark/build" \
     --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
     "make -j${threads}"

# generate cachefiles -- since we are testing gfx902 and 4 CUs (default config)
# in tester, we want cachefiles for this setup
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
    "-v${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
    --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
    "python3 generate_cachefiles.py cachefiles.csv --gfx-version=gfx902 \
    --num-cus=4"

# generate mmap data for DNNMark (makes simulation much faster)
docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
    --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
    "g++ -std=c++0x generate_rand_data.cpp -o generate_rand_data"

docker run --rm -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}/gem5-resources/src/gpu/DNNMark" hacc-test-weekly bash -c \
    "./generate_rand_data"

# now we can run DNNMark!
# DNNMark is representative of several simple (fast) layers within ML
# applications, which are heavily used in modern GPU applications.  So, we want
# to make sure support for these applications are tested.  Run three variants:
# fwd_softmax, bwd_bn, fwd_pool; these tests ensure we run a variety of ML kernels,
# including both inference and training
docker run --rm --volume "${gem5_root}":"${gem5_root}" -v \
       "${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
       -w "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       "${gem5_root}/build/${gpu_isa}/gem5.opt" "${gem5_root}/configs/example/apu_se.py" -n3 \
       --reg-alloc-policy=dynamic \
       --benchmark-root="${gem5_root}/gem5-resources/src/gpu/DNNMark/build/benchmarks/test_fwd_softmax" \
       -c dnnmark_test_fwd_softmax \
       --options="-config ${gem5_root}/gem5-resources/src/gpu/DNNMark/config_example/softmax_config.dnnmark \
       -mmap ${gem5_root}/gem5-resources/src/gpu/DNNMark/mmap.bin"

docker run --rm --volume "${gem5_root}":"${gem5_root}" -v \
       "${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
       -w "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       "${gem5_root}/build/${gpu_isa}/gem5.opt" "${gem5_root}/configs/example/apu_se.py" -n3 \
       --reg-alloc-policy=dynamic \
       --benchmark-root="${gem5_root}/gem5-resources/src/gpu/DNNMark/build/benchmarks/test_fwd_pool" \
       -c dnnmark_test_fwd_pool \
       --options="-config ${gem5_root}/gem5-resources/src/gpu/DNNMark/config_example/pool_config.dnnmark \
       -mmap ${gem5_root}/gem5-resources/src/gpu/DNNMark/mmap.bin"

docker run --rm --volume "${gem5_root}":"${gem5_root}" -v \
       "${gem5_root}/gem5-resources/src/gpu/DNNMark/cachefiles:/root/.cache/miopen/2.9.0" \
       -w "${gem5_root}/gem5-resources/src/gpu/DNNMark" \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       "${gem5_root}/build/${gpu_isa}/gem5.opt" "${gem5_root}/configs/example/apu_se.py" -n3 \
       --reg-alloc-policy=dynamic \
       --benchmark-root="${gem5_root}/gem5-resources/src/gpu/DNNMark/build/benchmarks/test_bwd_bn" \
       -c dnnmark_test_bwd_bn \
       --options="-config ${gem5_root}/gem5-resources/src/gpu/DNNMark/config_example/bn_config.dnnmark \
       -mmap ${gem5_root}/gem5-resources/src/gpu/DNNMark/mmap.bin"

# test HACC
# build HACC
docker run --rm -v ${PWD}:${PWD} -w \
       "${gem5_root}/gem5-resources/src/gpu/halo-finder/src" -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly make hip/ForceTreeTest

# Like LULESH, HACC is heavily used in the HPC community and is used to stress
# the GPU memory system
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/halo-finder/src/hip \
       -c ForceTreeTest --options="0.5 0.1 64 0.1 1 N 12 rcb"

# test Pannotia
# Pannotia has 6 different benchmarks (BC, Color, FW, MIS, PageRank, SSSP), of
# which 3 (Color, PageRank, SSSP) have 2 different variants.  Since they are
# useful for testing irregular GPU application behavior, we test each.

# build BC
docker run --rm -v ${PWD}:${PWD} \
       -w ${gem5_root}/gem5-resources/src/gpu/pannotia/bc -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; make gem5-fusion"

# # get input dataset for BC test
wget http://dist.gem5.org/dist/develop/datasets/pannotia/bc/1k_128k.gr
# run BC
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=gem5-resources/src/gpu/pannotia/bc/bin -c bc.gem5 \
       --options="1k_128k.gr"

# build Color Max
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/color -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; make gem5-fusion"

# run Color (Max) (use same input dataset as BC for faster testing)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/color/bin \
       -c color_max.gem5 --options="1k_128k.gr 0"

# build Color (MaxMin)
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/color -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; export VARIANT=MAXMIN ; make gem5-fusion"

# run Color (MaxMin) (use same input dataset as BC for faster testing)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/color/bin \
       -c color_maxmin.gem5 --options="1k_128k.gr 0"

# build FW
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/fw \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; make default; make gem5-fusion"

# create input mmap file for FW
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c\
       "./gem5-resources/src/gpu/pannotia/fw/bin/fw_hip ./gem5-resources/src/gpu/pannotia/fw/1k_128k.gr 1"

# run FW (use same input dataset as BC for faster testing)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/fw/bin \
       -c fw_hip.gem5 --options="1k_128k.gr 2"

# build MIS
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/mis -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; make gem5-fusion"

# run MIS (use same input dataset as BC for faster testing)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/mis/bin \
       -c mis_hip.gem5 --options="1k_128k.gr 0"

# build Pagerank Default variant
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/pagerank -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; make gem5-fusion"

# get PageRank input dataset
wget http://dist.gem5.org/dist/develop/datasets/pannotia/pagerank/coAuthorsDBLP.graph
# run PageRank (Default)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/pagerank/bin \
       -c pagerank.gem5 --options="coAuthorsDBLP.graph 1"

# build PageRank SPMV variant
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/pagerank -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; export VARIANT=SPMV ; make gem5-fusion"

# run PageRank (SPMV)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/pagerank/bin \
       -c pagerank_spmv.gem5 --options="coAuthorsDBLP.graph 1"

# build SSSP CSR variant
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/sssp -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; make gem5-fusion"

# run SSSP (CSR) (use same input dataset as BC for faster testing)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/sssp/bin \
       -c sssp.gem5 --options="1k_128k.gr 0"

# build SSSP ELL variant
docker run --rm -v ${gem5_root}:${gem5_root} -w \
       ${gem5_root}/gem5-resources/src/gpu/pannotia/sssp -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "export GEM5_PATH=${gem5_root} ; export VARIANT=ELL ; make gem5-fusion"

# run SSSP (ELL) (use same input dataset as BC for faster testing)
docker run --rm -v ${gem5_root}:${gem5_root} -w ${gem5_root} -u $UID:$GID \
       --memory="${docker_mem_limit}" hacc-test-weekly \
       ${gem5_root}/build/${gpu_isa}/gem5.opt \
       ${gem5_root}/configs/example/apu_se.py -n3 --mem-size=8GB \
       --reg-alloc-policy=dynamic \
       --benchmark-root=${gem5_root}/gem5-resources/src/gpu/pannotia/sssp/bin \
       -c sssp_ell.gem5 --options="1k_128k.gr 0"

# Delete the gem5 resources repo we created -- need to do in docker because of
# cachefiles DNNMark creates
docker run --rm --volume "${gem5_root}":"${gem5_root}" -w \
       "${gem5_root}" --memory="${docker_mem_limit}" hacc-test-weekly bash -c \
       "rm -rf ${gem5_root}/gem5-resources"

# Delete the gem5 m5out folder we created
rm -rf ${gem5_root}/m5out

# delete Pannotia datasets we downloaded and output files it created
rm -f coAuthorsDBLP.graph 1k_128k.gr result.out

# Run tests to ensure the DRAMSys integration is still functioning correctly.
if [ -d "${gem5_root}/ext/dramsys/DRAMSys" ]; then
    rm -r "${gem5_root}/ext/dramsys/DRAMSys"
fi

cd "${gem5_root}/ext/dramsys"
git clone --recursive git@github.com:tukl-msd/DRAMSys.git DRAMSys
cd DRAMSys
git checkout -b gem5 09f6dcbb91351e6ee7cadfc7bc8b29d97625db8f
cd "${gem5_root}"

rm -rf "${gem5_root}/build/ALL"

docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" --rm \
    ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
       scons build/ALL/gem5.opt -j${threads}

docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" --rm \
    ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
       ./build/ALL/gem5.opt \
       configs/example/gem5_library/dramsys/arm-hello-dramsys.py

docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" --rm \
    ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
       ./build/ALL/gem5.opt \
       configs/example/gem5_library/dramsys/dramsys-traffic.py

docker run -u $UID:$GID --volume "${gem5_root}":"${gem5_root}" -w \
    "${gem5_root}" --memory="${docker_mem_limit}" --rm \
    ghcr.io/gem5/ubuntu-22.04_all-dependencies:${tag} \
       ./build/ALL/gem5.opt \
       configs/example/dramsys.py
