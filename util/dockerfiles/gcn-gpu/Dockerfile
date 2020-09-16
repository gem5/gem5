FROM ubuntu:16.04

# Should be minimal needed packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    findutils \
    file \
    libunwind8 \
    libunwind-dev \
    pkg-config \
    build-essential \
    gcc-multilib \
    g++-multilib \
    git \
    ca-certificates \
    m4 \
    zlib1g \
    zlib1g-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libprotoc-dev \
    libgoogle-perftools-dev \
    python-dev \
    python \
    python-yaml \
    python-six \
    python-pip \
    wget \
    libpci3 \
    libelf1 \
    libelf-dev \
    cmake \
    openssl \
    libssl-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libboost-dev \
    libpng12-dev

RUN python -m pip install -U pip && \
    python -m pip install -U setuptools scons

ARG gem5_dist=http://dist.gem5.org/dist/develop

# Install ROCm 1.6 binaries
RUN wget -qO- ${gem5_dist}/apt_1.6.2.tar.bz2 \
    | tar -xjv \
    && cd apt_1.6.2/pool/main/ \
    && dpkg -i h/hsakmt-roct-dev/* \
    && dpkg -i h/hsa-ext-rocr-dev/* \
    && dpkg -i h/hsa-rocr-dev/* \
    && dpkg -i r/rocm-utils/* \
    && dpkg -i h/hcc/* \
    && dpkg -i r/rocm-opencl/* \
    && dpkg -i r/rocm-opencl-dev/*

# Get ROCm libraries we need to compile from source (and ROCm-profiler)
RUN git clone --single-branch https://github.com/ROCm-Developer-Tools/HIP/ && \
    git clone --single-branch https://github.com/ROCmSoftwarePlatform/hipBLAS/ && \
    git clone --single-branch https://github.com/ROCmSoftwarePlatform/rocBLAS/ && \
    git clone --single-branch https://github.com/ROCmSoftwarePlatform/MIOpenGEMM/ && \
    git clone --single-branch https://github.com/ROCmSoftwarePlatform/MIOpen/ && \
    git clone --single-branch https://github.com/RadeonOpenCompute/rocm-cmake/ && \
    git clone --single-branch https://github.com/rocmarchive/ROCm-Profiler.git

# Apply patches to various repos
RUN mkdir -p /patch && cd /patch && \
    wget ${gem5_dist}/rocm_patches/hipBLAS.patch && \
    wget ${gem5_dist}/rocm_patches/hip.patch_v2 && \
    wget ${gem5_dist}/rocm_patches/miopen-conv.patch && \
    wget ${gem5_dist}/rocm_patches/rocBLAS.patch

RUN git -C /HIP/ checkout 0e3d824e && git -C /HIP/ apply /patch/hip.patch_v2 && \
    git -C /hipBLAS/ checkout ee57787e && git -C /hipBLAS/ apply /patch/hipBLAS.patch && \
    git -C /rocBLAS/ checkout cbff4b4e && git -C /rocBLAS/ apply /patch/rocBLAS.patch && \
    git -C /MIOpenGEMM/ checkout 9547fb9e && \
    git -C /MIOpen/ checkout 01d6ca55c && git -C /MIOpen/ apply /patch/miopen-conv.patch

ENV ROCM_PATH /opt/rocm
ENV HCC_HOME ${ROCM_PATH}/hcc
ENV HSA_PATH ${ROCM_PATH}/hsa
ENV HIP_PATH ${ROCM_PATH}/hip
ENV HIP_PLATFORM hcc
ENV PATH ${ROCM_PATH}/bin:${HCC_HOME}/bin:${HSA_PATH}/bin:${HIP_PATH}/bin:${PATH}
ENV HCC_AMDGPU_TARGET gfx801

# Create build dirs for machine learning ROCm installs
RUN mkdir -p /HIP/build && \
    mkdir -p /rocBLAS/build && \
    mkdir -p /hipBLAS/build && \
    mkdir -p /rocm-cmake/build && \
    mkdir -p /MIOpenGEMM/build && \
    mkdir -p /MIOpen/build

# Do the builds, empty build dir to trim image size
WORKDIR /HIP/build
RUN cmake .. && make -j$(nproc) && make install && rm -rf *

WORKDIR /rocBLAS/build
RUN CXX=/opt/rocm/bin/hcc cmake -DCMAKE_CXX_FLAGS="--amdgpu-target=gfx801" .. && \
    make -j$(nproc) && make install && rm -rf *

WORKDIR /hipBLAS/build
RUN CXX=/opt/rocm/bin/hcc cmake -DCMAKE_CXX_FLAGS="--amdgpu-target=gfx801" .. && \
    make -j$(nproc) && make install && rm -rf *

WORKDIR /rocm-cmake/build
RUN cmake .. && cmake --build . --target install && rm -rf *

WORKDIR /MIOpenGEMM/build
RUN cmake .. && make miopengemm && make install && rm -rf *

# Should link this in as a volume if at all possible
RUN mkdir -p /.cache/miopen && chmod 777 /.cache/miopen

# Un-set default c++ version for MIOpen compilation
# As MIOpen 1.7 requires c++14 or higher
RUN sed -i 's/INTERFACE_COMPILE_OPTIONS "-std=c++amp;-fPIC;-gline-tables-only"/#&/' /opt/rocm/hcc-1.0/lib/cmake/hcc/hcc-targets.cmake && \
    sed -i 's/INTERFACE_COMPILE_OPTIONS "-hc"/#&/' /opt/rocm/hcc-1.0/lib/cmake/hcc/hcc-targets.cmake

WORKDIR /MIOpen
# Half is required; This is the version that MIOpen would download
RUN wget https://github.com/pfultz2/half/archive/1.12.0.tar.gz && \
    tar -xzf 1.12.0.tar.gz

WORKDIR /MIOpen/build
RUN CXX=/opt/rocm/hcc/bin/hcc cmake \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX=/opt/rocm \
    -DMIOPEN_BACKEND=HIP \
    -DCMAKE_PREFIX_PATH="/opt/rocm/hip;/opt/rocm/hcc;/opt/rocm/rocdl;/opt/rocm/miopengemm;/opt/rocm/hsa" \
    -DMIOPEN_CACHE_DIR=/.cache/miopen \
    -DMIOPEN_AMDGCN_ASSEMBLER_PATH=/opt/rocm/opencl/bin \
    -DHALF_INCLUDE_DIR=/MIOpen/half-1.12.0/include \
    -DCMAKE_CXX_FLAGS="-isystem /usr/include/x86_64-linux-gnu" .. && \
    make -j$(nproc) && make install && rm -rf *

# Re-set defaults
RUN sed -i 's/#\(INTERFACE_COMPILE_OPTIONS "-std=c++amp;-fPIC;-gline-tables-only"\)/\1/' /opt/rocm/hcc-1.0/lib/cmake/hcc/hcc-targets.cmake && \
    sed -i 's/#\(INTERFACE_COMPILE_OPTIONS "-hc"\)/\1/' /opt/rocm/hcc-1.0/lib/cmake/hcc/hcc-targets.cmake

# Create performance DB for gfx801.
WORKDIR /opt/rocm/miopen/share/miopen/db
RUN cp gfx803_64.cd.pdb.txt gfx801_4.cd.pdb.txt && \
    cp gfx803_64.cd.pdb.txt gfx801_8.cd.pdb.txt && \
    cp gfx803_64.cd.pdb.txt gfx801_16.cd.pdb.txt && \
    cp gfx803_64.cd.pdb.txt gfx801_32.cd.pdb.txt && \
    cp gfx803_64.cd.pdb.txt gfx801_64.cd.pdb.txt

# Install profiler from .deb file, works for 1.6.2
WORKDIR /ROCm-Profiler
RUN dpkg -i package/rocm-profiler_4.0.6036_amd64.deb

WORKDIR /
