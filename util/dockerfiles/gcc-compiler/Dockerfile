LABEL org.opencontainers.image.source=https://github.com/gem5/gem5
LABEL org.opencontainers.image.description="A Dockerfile to build an Ubuntu 24.04 image with all dependencies for gem5, with GCC 13."
LABEL org.opencontainers.image.licenses=BSD-3-Clause

FROM --platform=${BUILDPLATFORM} ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest

ARG version=13 # Version of GCC to install in this image. Default is 13.

RUN apt -y update && \
    apt -y install gcc-${version} g++-${version} && \
    update-alternatives --install \
        /usr/bin/g++ g++ /usr/bin/g++-${version} 100 && \
    update-alternatives --install \
        /usr/bin/gcc gcc /usr/bin/gcc-${version} 100 && \
    update-alternatives --install \
        /usr/bin/c++ c++ /usr/bin/g++-${version} 100 && \
    update-alternatives --install \
        /usr/bin/cc cc /usr/bin/gcc-${version} 100
