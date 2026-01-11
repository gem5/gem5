# Copyright (c) 2023 The Regents of the University of California
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

# docker buildx bake --push
# https://docs.docker.com/build/bake/reference

variable "IMAGE_URI" {
  default = "ghcr.io/gem5" # The gem5 GitHub container registry.
}

variable "CACHE_PREFIX" {
  # The cache prefix for the build cache.
  default = "type=registry,ref=${IMAGE_URI}"
}

variable "CACHE_TAG" {
  default = "build-cache" # The tag for the build cache.
}

variable "TAG" {
  default = "v25-1"
}

# Common attributes across all targets. Note: these can be overwritten.
target "common" {
  # Here we are enabling multi-platform builds. We are compiling to ARM64 and
  # AMD64 by default.
  platforms = ["linux/amd64", "linux/arm64"]
  pull = true
  dockerfile = "Dockerfile"
}

group "base-images" {
    targets = [
        "ubuntu-24-04_all-dependencies",
    ]
}

# A group of targets to be built. Note: groups can contain other groups.
# Any target or group can be build individually. I.e.:
# `docker buildx bake --push ubuntu-20-04_all-dependencies` or
# `docker buildx bake --push ubuntu-releases`.
group "default" {
  targets=[
    "qemu-riscv-env",
    "clang-compilers",
    "gcc-compilers",
    "ubuntu-releases",
    "gcn-gpu",
    "gpu-fs",
    "sst",
    "systemc",
    "devcontainer"
  ]
}

target "qemu-riscv-env" {
    inherits = ["common"]
    context = "qemu-riscv-env"
    annotations = ["index,manifest:org.opencontainers.image.description=An image capable of running a RISC-V QEMU simulation. Used to build RISC-V disk images for gem5-resources."]
    cache-from = ["${CACHE_PREFIX}/qemu-riscv-env:${CACHE_TAG}"]
    cache-to = ["${CACHE_PREFIX}/qemu-riscv-env:${CACHE_TAG}"]
    tags=["${IMAGE_URI}/qemu-riscv-env:${TAG}"]
}


group "clang-compilers" {
  targets = [
    "clang-version-14",
    "clang-version-15",
    "clang-version-16",
    "clang-version-17",
    "clang-version-18",
    "clang-version-19"
  ]
}

target "clang-version-14" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a Clang v14 compiler."]
  args = {
    version = "14"
  }
  context = "clang-compiler"
  cache-from = ["${CACHE_PREFIX}/clang-version-14:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/clang-version-14:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/clang-version-14:${TAG}"]
}

target "clang-version-15" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a Clang v15 compiler."]
  args = {
    version = "15"
  }
  context = "clang-compiler"
  cache-from = ["${CACHE_PREFIX}/clang-version-15:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/clang-version-15:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/clang-version-15:${TAG}"]
}

target "clang-version-16" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a Clang v16 compiler."]
  args = {
    version = "16"
  }
  context = "clang-compiler"
  cache-from = ["${CACHE_PREFIX}/clang-version-16:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/clang-version-16:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/clang-version-16:${TAG}"]
}

target "clang-version-17" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a Clang v17 compiler."]
  args = {
    version = "17"
  }
  context = "clang-compiler"
  cache-from = ["${CACHE_PREFIX}/clang-version-17:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/clang-version-17:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/clang-version-17:${TAG}"]
}

target "clang-version-18" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a Clang v18 compiler."]
  args = {
    version = "18"
  }
  context = "clang-compiler"
  cache-from = ["${CACHE_PREFIX}/clang-version-18:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/clang-version-18:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/clang-version-18:${TAG}"]
}

target "clang-version-19" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a Clang v19 compiler."]
  args = {
    version = "19"
  }
  context = "clang-compiler"
  cache-from = ["${CACHE_PREFIX}/clang-version-19:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/clang-version-19:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/clang-version-19:${TAG}"]
}

group "gcc-compilers" {
  targets = [
    "gcc-version-11",
    "gcc-version-12",
    "gcc-version-13",
    "gcc-version-14"
  ]
}
target "gcc-version-11" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a GCC v11 compiler."]
  args = {
    version = "11"
  }
  context = "gcc-compiler"
  cache-from = ["${CACHE_PREFIX}/gcc-version-11:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/gcc-version-11:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/gcc-version-11:${TAG}"]
}

target "gcc-version-12" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a GCC v12 compiler."]
  args = {
    version = "12"
  }
  context = "gcc-compiler"
  cache-from = ["${CACHE_PREFIX}/gcc-version-12:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/gcc-version-12:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/gcc-version-12:${TAG}"]
}

target "gcc-version-13" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a GCC v13 compiler."]
  args = {
    version = "13"
  }
  context = "gcc-compiler"
  cache-from = ["${CACHE_PREFIX}/gcc-version-13:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/gcc-version-13:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/gcc-version-13:${TAG}"]
}

target "gcc-version-14" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image with all dependencies for building gem5 with a GCC v14 compiler."]
  args = {
    version = "14"
  }
  context = "gcc-compiler"
  cache-from = ["${CACHE_PREFIX}/gcc-version-14:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/gcc-version-14:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/gcc-version-14:${TAG}"]
}

group "ubuntu-releases" {
  targets=[
    "ubuntu-24-04_all-dependencies",
    "ubuntu-22-04_all-dependencies",
    "ubuntu-24-04_min-dependencies"
  ]
}

target "ubuntu-24-04_all-dependencies" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An Ubuntu 24.04 image with all dependencies required for building and running gem5."]
  context = "ubuntu-24.04_all-dependencies"
  cache-from = ["${CACHE_PREFIX}/ubuntu-24.04_all-dependencies:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/ubuntu-24.04_all-dependencies:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/ubuntu-24.04_all-dependencies:${TAG}"]
}

target "ubuntu-22-04_all-dependencies" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An Ubuntu 22.04 image with all dependencies required for building and running gem5."]
  context = "ubuntu-22.04_all-dependencies"
  cache-from = ["${CACHE_PREFIX}/ubuntu-22.04_all-dependencies:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/ubuntu-22.04_all-dependencies:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/ubuntu-22.04_all-dependencies:${TAG}"]
}

target "ubuntu-24-04_min-dependencies" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An Ubuntu 24.04 image with the minimum dependencies required for building and running gem5."]
  context = "ubuntu-24.04_min-dependencies"
  cache-from = ["${CACHE_PREFIX}/ubuntu-24.04_min-dependencies:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/ubuntu-24.04_min-dependencies:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/ubuntu-24.04_min-dependencies:${TAG}"]
}

target "gcn-gpu" {
  inherits = ["common"]
  platforms = ["linux/amd64"] # Only build for x86.
  context = "gcn-gpu"
  cache-from = ["${CACHE_PREFIX}/gcn-gpu:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/gcn-gpu:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/gcn-gpu:${TAG}"]
}

target "gpu-fs" {
    inherits = ["common"]
    context = "gpu-fs"
    platforms = ["linux/amd64"] # Only build for x86.
    cache-from = ["${CACHE_PREFIX}/gpu-fs:${CACHE_TAG}"]
    cache-to = ["${CACHE_PREFIX}/gpu-fs:${CACHE_TAG}"]
    tags = ["${IMAGE_URI}/gpu-fs:${TAG}"]
}

target "sst" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image containing all the requirements for building and running gem5 in addition to SST. Used to test gem5-SST integration."]
  context = "sst"
  cache-from = ["${CACHE_PREFIX}/sst-env:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/sst-env:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/sst-env:${TAG}"]
}

target "systemc" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=An image containing all the requirements for building and running gem5 in addition to SystemC. Used to test gem5-SystemC integration."]
  context = "systemc"
  cache-from = ["${CACHE_PREFIX}/systemc-env:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/systemc-env:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/systemc-env:${TAG}"]
}

target "devcontainer" {
  inherits = ["common"]
  annotations = ["index,manifest:org.opencontainers.image.description=A devcontainer image for gem5 development referenced in the repo's ./devcontainer/devcontainer.json file. Includes all dependencies required for gem5 development."]
  dependencies = ["devcontainer"]
  context = "devcontainer"
  cache-from = ["${CACHE_PREFIX}/devcontainer:${CACHE_TAG}"]
  cache-to = ["${CACHE_PREFIX}/devcontainer:${CACHE_TAG}"]
  tags = ["${IMAGE_URI}/devcontainer:${TAG}"]
}
