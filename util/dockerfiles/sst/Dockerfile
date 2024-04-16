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

FROM --platform=${BUILDPLATFORM} ghcr.io/gem5/ubuntu-20.04_all-dependencies:latest

ENV DEBIAN_FRONTEND=noninteractive

ENV PYTHON_CONF=/usr/bin/python3-config
ENV SST_CORE_HOME="/sst"
ENV SST_CORE_BUILD_TMP="/var/tmp/sst-core"
ENV SST_ELEMENTS_BUILD_TMP="/var/tmp/sst-elements"

RUN mkdir -p ${SST_CORE_HOME}

# Download, build and install SST-Core without MPI support.
# The downloaded files and build directory are removed in the same layer to
# reduce the size of the layer. 
RUN mkdir -p "${SST_CORE_BUILD_TMP}" && \
    cd "${SST_CORE_BUILD_TMP}" && \
    wget https://github.com/sstsimulator/sst-core/releases/download/v13.0.0_Final/sstcore-13.0.0.tar.gz && \
    tar xf sstcore-13.0.0.tar.gz && \
    cd sstcore-13.0.0 && \
   ./configure --prefix="${SST_CORE_HOME}" \
               --with-python="${PYTHON_CONF}" \
               --disable-mpi && \
    make all -j $(nproc) && \
    make install &&  \
    rm -rf "${PYTHON_CORE_BUILD_TMP}"

# Download, build and install  SST-Elements.
# The downloaded files and build directory are removed in the same layer to
# reduce the size of the layer.
RUN mkdir -p "${SST_ELEMENTS_BUILD_TMP}" && \
    cd "${SST_ELEMENTS_BUILD_TMP}" && \
    wget https://github.com/sstsimulator/sst-elements/releases/download/v13.0.0_Final/sstelements-13.0.0.tar.gz && \
    tar xf sstelements-13.0.0.tar.gz && \
    cd sst-elements-library-13.0.0 && \
    ./configure --prefix="${SST_CORE_HOME}" \
                --with-python="${PYTHON_CONF}" \
                --with-sst-core="${SST_CORE_HOME}" && \
    make all -j $(nproc) && \
    make install && \
    rm -rf "${SST_ELEMENTS_BUILD_TMP}"

# Setting the environmental variabl
ENV PATH=$PATH:$SST_CORE_HOME/bin
ENV PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$SST_CORE_HOME/lib/pkgconfig/
