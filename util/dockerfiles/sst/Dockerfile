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

FROM --platform=${BUILDPLATFORM} ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update && apt -y upgrade && \
    apt -y install build-essential git m4 scons zlib1g zlib1g-dev \
    libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
    python3-dev python-is-python3 doxygen libboost-all-dev \
    libhdf5-serial-dev python3-pydot libpng-dev libelf-dev pkg-config pip \
    python3-venv wget

ENV SST_CORE_HOME="/sst/"

# Use GCC 10
RUN apt update && apt -y install gcc-10 g++-10
RUN update-alternatives --install \
    /usr/bin/g++ g++ /usr/bin/g++-10 100
RUN update-alternatives --install \
    /usr/bin/gcc gcc /usr/bin/gcc-10 100
RUN update-alternatives --install \
    /usr/bin/c++ c++ /usr/bin/g++-10 100
RUN update-alternatives --install \
    /usr/bin/cc cc /usr/bin/gcc-10 100

# SST Stuff
RUN mkdir /sst

# Download and build SST-Core without MPI support
WORKDIR /sst/
RUN wget https://github.com/sstsimulator/sst-core/releases/download/v13.0.0_Final/sstcore-13.0.0.tar.gz; \
    tar xf sstcore-13.0.0.tar.gz;
WORKDIR /sst/sstcore-13.0.0/
RUN ./configure --prefix=$SST_CORE_HOME --with-python=/usr/bin/python3-config \
                                        --disable-mpi; \
    make all -j $(nproc); \
    make install;

# Download and build SST-Elements
WORKDIR /sst
RUN wget https://github.com/sstsimulator/sst-elements/releases/download/v13.0.0_Final/sstelements-13.0.0.tar.gz; \
    tar xf sstelements-13.0.0.tar.gz;
WORKDIR /sst/sst-elements-library-13.0.0/
RUN ./configure --prefix=$SST_CORE_HOME --with-python=/usr/bin/python3-config \
                                        --with-sst-core=$SST_CORE_HOME; \
    make all -j $(nproc); \
    make install;

# Setting the environmental variables
ENV PATH=$PATH:$SST_CORE_HOME/bin
ENV PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$SST_CORE_HOME/lib/pkgconfig/
