# Copyright (c) 2020 The Regents of the University of California
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

# Valid version values:
# 7
# 8
# 9 # Not supported. See: https://github.com/gem5/gem5/issues/555.
# 10
ARG version

ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update && apt -y upgrade && \
    apt -y install git m4 scons zlib1g zlib1g-dev libprotobuf-dev \
    protobuf-compiler libprotoc-dev libgoogle-perftools-dev python3-dev \
    python-is-python3 doxygen libboost-all-dev libhdf5-serial-dev \
    python3-pydot libpng-dev gcc-${version} g++-${version} make

RUN update-alternatives --install \
    /usr/bin/g++ g++ /usr/bin/g++-${version} 100
RUN update-alternatives --install \
    /usr/bin/gcc gcc /usr/bin/gcc-${version} 100
RUN update-alternatives --install \
    /usr/bin/c++ c++ /usr/bin/g++-${version} 100
RUN update-alternatives --install \
    /usr/bin/cc cc /usr/bin/gcc-${version} 100
