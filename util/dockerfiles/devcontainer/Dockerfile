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

# As this is a multi-platform image, we must build it using the
# "docker-bake.hcl" specification via `docker buildx`. To do so, execute the
# following:
#
# `docker buildx bake devcontainer --push`

# Keep the development environment independent of the demo executable.
ARG BASE_IMAGE=ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest
FROM ${BASE_IMAGE} AS development

LABEL org.opencontainers.image.source=https://github.com/gem5/gem5
LABEL org.opencontainers.image.licenses=BSD-3-Clause

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    gdb \
    unzip && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

# Build a release executable only for the optional demonstration image.
FROM ${BASE_IMAGE} AS release-builder
ARG GEM5_VERSION=25.1.0.1
ARG BUILD_JOBS=2
RUN wget -O /tmp/gem5.tar.gz \
        "https://github.com/gem5/gem5/archive/refs/tags/v${GEM5_VERSION}.tar.gz" && \
    mkdir /gem5 && \
    tar xzf /tmp/gem5.tar.gz -C /gem5 --strip-components=1 && \
    rm /tmp/gem5.tar.gz
WORKDIR /gem5
RUN scons build/ALL/gem5.fast -j"${BUILD_JOBS}"

FROM development AS demo
ARG GEM5_VERSION=25.1.0.1
LABEL org.opencontainers.image.version=${GEM5_VERSION}
COPY --from=release-builder /gem5/build/ALL/gem5.fast \
    /usr/local/bin/gem5-release

# Tools for building guest workloads and full-system disk images.
FROM development AS workloads
RUN apt-get update && apt-get install -y --no-install-recommends \
    qemu-system \
    qemu-utils \
    g++-aarch64-linux-gnu \
    g++-riscv64-linux-gnu \
    g++-x86-64-linux-gnu && \
    rm -rf /var/lib/apt/lists/*
