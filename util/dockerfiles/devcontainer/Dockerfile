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

LABEL org.opencontainers.image.source=https://github.com/gem5/gem5
LABEL org.opencontainers.image.description="This Dockerfile creates a devcontainer for gem5 development referenced in ./devcontainer/devcontainer.json. The devcontainer is a multi-platform container that can be built for x86, arm, and riscv and includes all dependencies required for gem5 development."
LABEL org.opencontainers.image.licenses=BSD-3-Clause

# As this is a multi-platform image, we must build it using the
# "docker-bake.hcl" specification via `docker buildx`. To do so, execute the
# following:
#
# `docker buildx bake devcontainer --push`

# Stage 1: We build the stable version of gem5.
# In the final stage this this is copied into /usr/local/bin/gem5. This ensures
# there is a pre-built gem5 binary in each devcontainer. This can save time
# if the container is used for education or demonstration purposes.
FROM ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest as builder
RUN wget https://github.com/gem5/gem5/archive/refs/tags/v24.0.tar.gz && \
    tar xzf v24.0.tar.gz && mv gem5-24.0 gem5 && rm v24.0.tar.gz
WORKDIR /gem5
RUN scons build/ALL/gem5.opt -j`nproc`
RUN scons defconfig build/ALL_CHI build_opts/ALL && \
    scons setconfig build/ALL_CHI RUBY_PROTOCOL_CHI=y && \
    scons build/ALL_CHI/gem5.opt -j`nproc`
RUN scons build/VEGA_X86/gem5.opt -j`nproc`

# Stage 2: Create the devcontainer image.
# This includes all dependencies for building and running gem5, the
# dependencies for developing gem5, and a pre-built gem5 binary.
FROM ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest
COPY --from=builder /gem5/build/ALL/gem5.opt /usr/local/bin/gem5-mesi
COPY --from=builder /gem5/build/ALL_CHI/gem5.opt /usr/local/bin/gem5-chi
COPY --from=builder /gem5/build/VEGA_X86/gem5.opt /usr/local/bin/gem5-vega
RUN ln -s /usr/local/bin/gem5-chi /usr/local/bin/gem5

# Stage 3: Install additional packages.
# While the base image includes all dependencies for building and running gem5,
# we install packages which are useful when running in a dev container and/or
# developing gem5. (e.g. vim to edit files, qemu for building disk images,
# etc.).
RUN apt -y update  && \
    apt -y install \
    vim \
    qemu-system \
    qemu-kvm \
    g++-aarch64-linux-gnu \
    g++-riscv64-linux-gnu \
    gcc-aarch64-linux-gnu \
    gcc-riscv64-linux-gnu \
    locales \
    gdb \
    unzip

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
