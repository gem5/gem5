# Copyright (c) 2021 Kyle Roarty
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
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update
RUN apt -y upgrade
RUN apt -y install build-essential git m4 scons zlib1g zlib1g-dev \
    libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
    python3-dev python3-six python-is-python3 doxygen libboost-all-dev \
    libhdf5-serial-dev python3-pydot libpng-dev libelf-dev pkg-config

# Requirements for ROCm
RUN apt -y install cmake mesa-common-dev libgflags-dev libgoogle-glog-dev

# Needed to get ROCm repo, build packages
RUN apt -y install wget gnupg2 rpm

RUN wget -q -O - https://repo.radeon.com/rocm/rocm.gpg.key | apt-key add -

# ROCm webpage says to use debian main, but the individual versions
# only have xenial
RUN echo 'deb [arch=amd64] https://repo.radeon.com/rocm/apt/4.0.1/ xenial main' | tee /etc/apt/sources.list.d/rocm.list

RUN apt-get update && apt -y install hsakmt-roct hsakmt-roct-dev
RUN ln -s /opt/rocm-4.0.1 /opt/rocm

RUN git clone -b rocm-4.0.0 https://github.com/RadeonOpenCompute/ROCR-Runtime.git && \
    mkdir -p /ROCR-Runtime/src/build

WORKDIR /ROCR-Runtime/src/build
# need MEMFD_CREATE=OFF as MEMFD_CREATE syscall isn't implemented
RUN cmake -DIMAGE_SUPPORT=OFF -DHAVE_MEMFD_CREATE=OFF \
    -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && make package
RUN apt -y install ./hsa-rocr-dev*.deb
WORKDIR /

# Dependencies for ROCclr
RUN apt -y install llvm-amdgpu libncurses5 libtinfo-dev rocm-device-libs comgr

RUN git clone -b rocm-4.0.0 \
    https://github.com/ROCm-Developer-Tools/ROCclr.git && \
    mkdir -p ROCclr/build

RUN git clone -b rocm-4.0.0 \
    https://github.com/RadeonOpenCompute/ROCm-OpenCL-Runtime.git && \
    mkdir -p ROCm-OpenCL-Runtime/build

WORKDIR /ROCclr
# The patch allows us to avoid building blit kernels on-the-fly in gem5
RUN wget -q -O - dist.gem5.org/dist/develop/rocm_patches/ROCclr.patch | git apply -v

WORKDIR /ROCclr/build
RUN cmake -DOPENCL_DIR="/ROCm-OpenCL-Runtime" \
    -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install
WORKDIR /

WORKDIR ROCm-OpenCL-Runtime/build
RUN cmake -DUSE_COMGR_LIBRARY=ON -DCMAKE_PREFIX_PATH="/opt/rocm" \
    -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make package
RUN apt -y install ./rocm-opencl-2.0.0-amd64.deb ./rocm-opencl-dev-2.0.0-amd64.deb
WORKDIR /

RUN git clone -b rocm-4.0.0 \
    https://github.com/ROCm-Developer-Tools/HIP.git && mkdir -p HIP/build

WORKDIR HIP/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DHSA_PATH=/usr/hsa \
    -DHIP_COMPILER=clang -DHIP_PLATFORM=rocclr -DCMAKE_PREFIX_PATH="/opt/rocm"\
    .. && make -j$(nproc) && make package
RUN apt -y install ./hip-base*.deb ./hip-rocclr*.deb
# These files here are needed but don't get installed through the .deb file,
# even though they seem to be included in the packaging, so symlink them
RUN ln -s /HIP/build/rocclr/CMakeFiles/Export/_opt/rocm/hip/lib/cmake/hip/* /opt/rocm/hip/lib/cmake/hip/
WORKDIR /

# rocBLAS downloads the most recent rocm-cmake if it isn't installed before
# building
RUN apt install rocm-cmake

RUN git clone -b rocm-4.0.0 \
    https://github.com/ROCmSoftwarePlatform/rocBLAS.git && mkdir rocBLAS/build

ENV HCC_AMDGPU_TARGET=gfx801
WORKDIR rocBLAS
# rocBLAS needs to be built from source otherwise gfx801 gets an error in HIP
# about there being no GPU binary available
RUN ./install.sh -d -a all -i
WORKDIR /

# MIOpen dependencies + MIOpen
RUN apt install rocm-clang-ocl miopen-hip

# Clone MIOpen repo so that we have the kernel sources available
RUN git clone -b rocm-4.0.1 https://github.com/ROCmSoftwarePlatform/MIOpen.git

# Make the MIOpen cache dir ahead of time and symlink for easier access
# when linking in the database file
RUN mkdir -p /root/.cache/miopen/2.9.0.8252-rocm-rel-4.0-26-64506314 && \
    ln -s /root/.cache/miopen/2.9.0.8252-rocm-rel-4.0-26-64506314 /root/.cache/miopen/2.9.0
