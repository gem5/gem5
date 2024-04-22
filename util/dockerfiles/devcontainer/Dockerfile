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

# This Dockerfile creates a devcontainer for gem5 development referenced in
# ./devcontainer/devcontainer.json. The devcontainer is a multi-platform
# container that can be built for x86, arm, and riscv and includes all
# dependencies required for gem5 development.
#
# As this is a multi-platform image, we must build it using the
# "docker-bake.hcl" specification via `docker buildx`. To do so, execute the
# following:
#
# `docker buildx bake devcontainer --push`

# Stage 1: We create the base image will all-dependencies.
# Notes:
# * This is an exact copy of the ubuntu-22.04_all-dependencies Dockerfile
#   We do not use the image because it is not multi-platform right now. When
#   the image utilizes multi-platform we can remove this duplication and
#   replace it with
#   `FROM ubuntu-22.04_all-dependencies:latest as all-dependenices`.
# * The all-dependencies docker is all dependeices needed to build and run
#   gem5. As there may be addition dependenices useful for development, we will
#   add in the final stage of the Dockerfile.
FROM --platform=${BUILDPLATFORM} ubuntu:22.04 as all-dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update && apt -y upgrade && \
    apt -y install build-essential git m4 scons zlib1g zlib1g-dev \
    libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
    python3-dev doxygen libboost-all-dev libhdf5-serial-dev python3-pydot \
    libpng-dev libelf-dev pkg-config pip python3-venv black python3-tk wget
RUN pip install mypy pre-commit
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0.tar.gz \
    && tar -xzf cmake-3.24.0.tar.gz \
    && cd cmake-3.24.0 \
    && ./bootstrap \
    && make -j`nproc` \
    && make install \
    && cd .. \
    && rm -rf cmake-3.24.0.tar.gz cmake-3.24.0

# Stage 2: We build the stable version of gem5.
# In the final stage this this is copied into /usr/local/bin/gem5. This ensures
# there is a pre-built gem5 binary in each devcontainer. This can save time
# if the container is used for education or demonstration purposes.
FROM --platform=${BUILDPLATFORM} all-dependencies as builder
RUN git clone --branch stable https://github.com/gem5/gem5
WORKDIR /gem5
RUN scons build/ALL/gem5.opt -j`nproc`

# Stage 3: The final stage where we create the devcontainer image.
# This includes all dependencies for building and running gem5, the
# dependencies for developing gem5, and a pre-built gem5 binary.
FROM --platform=${BUILDPLATFORM} all-dependencies
RUN apt -y update  && \
    apt -y install vim
COPY --from=builder /gem5/build/ALL/gem5.opt /usr/local/bin/gem5
