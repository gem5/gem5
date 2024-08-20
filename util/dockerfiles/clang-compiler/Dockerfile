FROM --platform=${BUILDPLATFORM} ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest

ARG version=16 # Version of Clang to install in this image. Default is 16.

RUN apt -y update && \
    apt -y install clang-${version} && \
    update-alternatives --install \
        /usr/bin/clang++ clang++ /usr/bin/clang++-${version} 100 && \
    update-alternatives --install \
        /usr/bin/clang clang /usr/bin/clang-${version} 100 && \
    update-alternatives --install \
        /usr/bin/c++ c++ /usr/bin/clang++-${version} 100 && \
    update-alternatives --install \
        /usr/bin/cc cc /usr/bin/clang-${version} 100
