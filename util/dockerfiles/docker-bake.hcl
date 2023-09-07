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

variable "TAG" {
  default = "latest"
}

# A group of targets to be built. Note: groups can contain other groups.
# Any target or group can be build individually. I.e.:
# `docker buildx bake --push ubuntu-20-04_all-dependencies` or
# `docker buildx bake --push ubuntu-releases`.
group "default" {
  targets=["clang-compilers", "ubuntu-releases"]
}

group "ubuntu-releases" {
    targets=["ubuntu-22-04_all-dependencies", "ubuntu-20-04_all-dependencies"]
}

# Common attributes across all targets. Note: these can be overwritten.
target "common" {
  # Here we are enabling multi-platform builds. We are compiling to both ARM
  # amd X86.
  platforms = ["linux/amd64", "linux/arm64"]
}

target "clang-compilers" {
    name="clang-compilers-${replace(ver, ".", "-")}"
    inherits = ["common"]
    context = "ubuntu-20.04_clang-version"
    dockerfile = "Dockerfile"
    matrix = {
        ver = ["6.0","7","8","9","10","11"]
    }
    args = {
        version=ver
    }
    tags = ["${IMAGE_URI}/clang-version-${ver}:${TAG}"]
}

target "ubuntu-22-04_all-dependencies" {
  inherits = ["common"]
  dockerfile = "Dockerfile"
  context = "ubuntu-22.04_all-dependencies"
  tags = ["${IMAGE_URI}/ubuntu-22.04_all-dependencies:${TAG}"]
}

target "ubuntu-20-04_all-dependencies" {
  inherits = ["common"]
  dockerfile = "Dockerfile"
  context = "ubuntu-20.04_all-dependencies"
  tags = ["${IMAGE_URI}/ubuntu-20.04_all-dependencies:${TAG}"]
}
