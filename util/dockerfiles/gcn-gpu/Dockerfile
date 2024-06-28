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

FROM --platform=${BUILDPLATFORM} ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt -y update && \
    apt -y install \
    # General gem5 requirements when building/running in Ubuntu 20.04, also
    # some utilities needed to compile and obtain the right packages that
    # make up this Docker image.
    build-essential git m4 scons zlib1g zlib1g-dev \
    libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
    python3-dev python-is-python3 doxygen libboost-all-dev \
    libhdf5-serial-dev python3-pydot libpng-dev libelf-dev pkg-config gdb \
    cmake wget gnupg2 rpm \
    # Requirements for ROCm
    mesa-common-dev libgflags-dev libgoogle-glog-dev \
    # ROCm dependencies and requirements to get ROCm repo and build packages.
    # Upgrade to gcc 10: gem5 supports gcc 10+.
    gcc-10 g++-10 cpp-10

# Update the default compiler to gcc 10
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 && \
    update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-10 100 && \
    update-alternatives --install /usr/bin/cc cc /usr/bin/gcc-10 100

# Add the ROCm apt key.
RUN wget -q -O - https://repo.radeon.com/rocm/rocm.gpg.key | apt-key add -

# ROCm webpage says to use debian main, but the individual versions
# only have xenial. We set the APT source here to obtain thecorrect
# ROCm packages.
RUN echo 'deb [arch=amd64] https://repo.radeon.com/rocm/apt/4.0.1/ xenial main' | \
    tee /etc/apt/sources.list.d/rocm.list

# Install/apt/4.0.1 packages.
RUN apt -y update && apt -y install hsakmt-roct hsakmt-roct-dev

# Create symbolic link: /opt/rocm -> /opt/rocm-4.0.1.
RUN ln -s /opt/rocm-4.0.1 /opt/rocm

# Download, build, and install the ROCR-Runtime.
# Sources and build files deleted after install to reduce the size of this
# layer.
RUN git clone -b rocm-4.0.0 --depth=1 \
        https://github.com/RadeonOpenCompute/ROCR-Runtime.git && \
    mkdir -p /ROCR-Runtime/src/build && \
    cd /ROCR-Runtime/src/build && \
    # We need MEMFD_CREATE=OFF as MEMFD_CREATE syscall isn't implemented.
    cmake -DIMAGE_SUPPORT=OFF -DHAVE_MEMFD_CREATE=OFF -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make package && \
    apt -y install ./hsa-rocr-dev*.deb && \
    # Cleanup
    cd / && \
    rm -rf /ROCR-Runtime

# This giant run encapsulates the cloning, building, installation of HIP,
# ROCm-OpenCL and ROCclr. These all depend upon one another's sources and build
# files, ergo they are all pulled, built, installed in a single RUN command.
# Once all three are installed their sources and build files are deleted.
RUN git clone -b rocm-4.0.0 --depth=1 \
        https://github.com/ROCm-Developer-Tools/HIP.git && \
    git clone -b rocm-4.0.0 --depth=1 \
        https://github.com/RadeonOpenCompute/ROCm-OpenCL-Runtime.git && \
    git clone -b rocm-4.0.0 --depth=1 \
        https://github.com/ROCm-Developer-Tools/ROCclr.git && \
    # Get ROCclr dependencies
    apt -y update && \
    apt -y install llvm-amdgpu libncurses5 libtinfo-dev rocm-device-libs \
                   comgr && \
    mkdir -p ROCclr/build && \
    cd ROCclr && \
    # The patch allows us to avoid building blit kernels on-the-fly in gem5
    wget -q -O - dist.gem5.org/dist/develop/rocm_patches/ROCclr.patch | \
        git apply -v && \
    # Build and install ROCclr.
    cd /ROCclr/build && \
    cmake \
        -DOPENCL_DIR="/ROCm-OpenCL-Runtime" \
        -DCMAKE_BUILD_TYPE=Release ..  && \
    make -j$(nproc) && \
    make install && \
    # We apply a patch to avoid a linking error -- "multiple definition of
    # 'ret_val'".
    # Issue here: https://github.com/ROCm/ROCm-OpenCL-Runtime/issues/113
    cd /ROCm-OpenCL-Runtime/khronos/icd && \
    wget -q -O - https://github.com/KhronosGroup/OpenCL-ICD-Loader/pull/101/commits/319ba95eb08aa7c622efae50cb62c7cd7de14c1b.patch | \
        patch -p1 && \
    # Build and install ROCm OpenCL.
    cd /ROCm-OpenCL-Runtime && \
    mkdir build && \
    cd build && \
    cmake \
        -DUSE_COMGR_LIBRARY=ON \
        -DCMAKE_PREFIX_PATH="/opt/rocm" \
        -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make package && \
    apt -y install ./rocm-opencl-2.0.0-amd64.deb \
                   ./rocm-opencl-dev-2.0.0-amd64.deb && \
    # Build and install HIP.
    mkdir -p /HIP/build && \
    cd /HIP/build && \
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DHSA_PATH=/usr/hsa \
        -DHIP_COMPILER=clang \
        -DHIP_PLATFORM=rocclr \
        -DCMAKE_PREFIX_PATH="/opt/rocm" .. && \
    make -j$(nproc) && \
    make package && \
    apt -y install ./hip-base*.deb ./hip-rocclr*.deb && \
    cp -r /HIP/build/rocclr/CMakeFiles/Export/_opt/rocm/hip/lib/cmake/hip/* \
          /opt/rocm/hip/lib/cmake/hip/ && \
    # Cleanup.
    cd / && \
    rm -rf /HIP /ROCclr /ROCm-OpenCL-Runtime


# Clone, build, and install rocBLAS.
# Sources and build files are deleted after the install.
# rocBLAS needs to be built from source otherwise certain gfx versions get errors in HIP
# about there being no GPU binary available
ENV HCC_AMDGPU_TARGET=gfx900,gfx902
RUN git clone -b rocm-4.0.0 --depth=1 \
        https://github.com/ROCmSoftwarePlatform/rocBLAS.git && \
    # rocBLAS downloads the most recent rocm-cmake if it isn't installed before
    # building
    apt -y update && \
    apt -y install rocm-cmake && \
    mkdir rocBLAS/build && \
    cd rocBLAS && \
    ./install.sh -d -i && \
    cd / && \
    rm -rf rocBLAS

# MIOpen dependencies + MIOpen
RUN apt -y update && \
    apt -y install rocm-clang-ocl miopen-hip

# Clone MIOpen repo so that we have the kernel sources available
RUN git clone -b rocm-4.0.1 --depth=1 \
    https://github.com/ROCmSoftwarePlatform/MIOpen.git

# Make the MIOpen cache dir ahead of time and symlink for easier access
# when linking in the database file
RUN mkdir -p /root/.cache/miopen/2.9.0.8252-rocm-rel-4.0-26-64506314
RUN ln -s /root/.cache/miopen/2.9.0.8252-rocm-rel-4.0-26-64506314 \
        /root/.cache/miopen/2.9.0

# Add commands from halofinder Dockerfile
RUN apt -y update && \
    apt -y install libopenmpi-dev libomp-dev

ENV HIPCC_BIN=/opt/rocm/bin
ENV MPI_INCLUDE=/usr/lib/x86_64-linux-gnu/openmpi/include

ENV OPT="-O3 -g -DRCB_UNTHREADED_BUILD -DUSE_SERIAL_COSMO"
ENV OMP="-I/usr/lib/llvm-10/include/openmp -L/usr/lib/llvm-10/lib -fopenmp"

ENV HIPCC_FLAGS="-v -ffast_math -DINLINE_FORCE -I${MPI_INCLUDE}"
ENV HIPCC_FLAGS="-v -I${MPI_INCLUDE} -I/opt/rocm/hip/include"

ENV HACC_PLATFORM="hip"
ENV HACC_OBJDIR="${HACC_PLATFORM}"

ENV HACC_CFLAGS="$OPT $OMP $HIPCC_FLAGS"
ENV HACC_CC="${HIPCC_BIN}/hipcc -x c -Xclang -std=c99"

ENV HACC_CXXFLAGS="$OPT $OMP $HIPCC_FLAGS"
ENV HACC_CXX="${HIPCC_BIN}/hipcc -Xclang"

ENV HACC_LDFLAGS="-lm -lrt"

# USE_SERIAL_COSMO must be set to avoid building the code with MPI, which isn't
# supported on the GPU model in gem5.
ENV USE_SERIAL_COSMO="1"
ENV HACC_NUM_CUDA_DEV="1"
ENV HACC_MPI_CFLAGS="$OPT $OMP $HIPCC_FLAGS"
ENV HACC_MPI_CC="${HIPCC_BIN}/hipcc -x c -Xclang -std=c99 -Xclang -pthread"

ENV HACC_MPI_CXXFLAGS="$OPT $OMP $HIPCC_FLAGS"
ENV HACC_MPI_CXX="${HIPCC_BIN}/hipcc -Xclang -pthread"
ENV HACC_MPI_LD="${HIPCC_BIN}/hipcc -Xclang -pthread"

ENV HACC_MPI_LDFLAGS="-lm -lrt"
