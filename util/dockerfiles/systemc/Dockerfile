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
FROM --platform=${BUILDPLATFORM} ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest
LABEL org.opencontainers.image.source=https://github.com/gem5/gem5
LABEL org.opencontainers.image.description="A Dockerfile for using SystemC with gem5."
LABEL org.opencontainers.image.licenses=BSD-3-Clause

ENV CXX="g++"
ENV CXXFLAGS="-std=c++17"

RUN wget https://www.accellera.org/images/downloads/standards/systemc/systemc-2.3.3.tar.gz; \
    tar xf systemc-2.3.3.tar.gz && \
    rm systemc-2.3.3.tar.gz && \
    cd systemc-2.3.3 && \
    mkdir objdir && \
    cd objdir && \
    ../configure --prefix=/opt/systemc && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf systemc-2.3.3
