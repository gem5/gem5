# Copyright (c) 2024 The Regents of the University of California
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
FROM --platform=${BUILDPLATFORM} ubuntu:22.04

# Valid version values:
# 13

ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update && \
    # software-properties-common is necessary to install
    # to add PPA to the system
    apt -y install software-properties-common && \
    # Installing PPA is necessary to install gcc-13
    # because it is not available in the default repositories
    # for Ubuntu 22.04
    add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt -y update && \
    apt -y install git m4 scons zlib1g zlib1g-dev libprotobuf-dev \
    protobuf-compiler libprotoc-dev libgoogle-perftools-dev python3-dev \
    doxygen libboost-all-dev libhdf5-serial-dev python3-pydot libpng-dev \
    gcc-13 g++-13 make

RUN update-alternatives --install \
    /usr/bin/g++ g++ /usr/bin/g++- 100
RUN update-alternatives --install \
    /usr/bin/gcc gcc /usr/bin/gcc-13 100
RUN update-alternatives --install \
    /usr/bin/c++ c++ /usr/bin/g++-13 100
RUN update-alternatives --install \
    /usr/bin/cc cc /usr/bin/gcc-13 100
