ARG base=ghcr.io/gem5/ubuntu-24.04_all-dependencies:latest
FROM ${base}

LABEL org.opencontainers.image.source=https://github.com/gem5/gem5
LABEL org.opencontainers.image.licenses=BSD-3-Clause

ARG version=16 # Version of Clang to install in this image. Default is 16.
# Optional GNU C++ library major to install alongside Clang.
ARG libstdcxx_version

RUN apt -y update && \
    apt -y install clang-${version} \
        ${libstdcxx_version:+libstdc++-${libstdcxx_version}-dev} && \
    update-alternatives --install \
        /usr/bin/clang++ clang++ /usr/bin/clang++-${version} 100 && \
    update-alternatives --install \
        /usr/bin/clang clang /usr/bin/clang-${version} 100 && \
    update-alternatives --install \
        /usr/bin/c++ c++ /usr/bin/clang++-${version} 100 && \
    update-alternatives --install \
        /usr/bin/cc cc /usr/bin/clang-${version} 100

ENV CC=clang \
    CXX=clang++
