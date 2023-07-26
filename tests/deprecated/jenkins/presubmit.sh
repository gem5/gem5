#!/bin/bash
#
# Copyright (c) 2018 The Regents of the University of California
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

set -e

DOCKER_IMAGE_ALL_DEP=gcr.io/gem5-test/ubuntu-22.04_all-dependencies:latest
DOCKER_IMAGE_CLANG_COMPILE=gcr.io/gem5-test/clang-version-14:latest
PRESUBMIT_STAGE2=tests/jenkins/presubmit-stage2.sh
GEM5ART_TESTS=tests/jenkins/gem5art-tests.sh

# Move the docker base directory to tempfs.
sudo /etc/init.d/docker stop
sudo mv /var/lib/docker /tmpfs/
sudo ln -s /tmpfs/docker /var/lib/docker
sudo /etc/init.d/docker start

# Move the CWD to the gem5 checkout.
cd git/jenkins-gem5-prod/

#  Using a docker image with all the dependencies, we run the gem5art tests.
docker run -u $UID:$GID --volume $(pwd):$(pwd) -w $(pwd) --rm \
    "${DOCKER_IMAGE_ALL_DEP}" "${GEM5ART_TESTS}"

#  Using a docker image with all the dependencies, we run the presubmit tests.
docker run -u $UID:$GID --volume $(pwd):$(pwd) -w $(pwd) --rm \
    "${DOCKER_IMAGE_ALL_DEP}" "${PRESUBMIT_STAGE2}"

# DOCKER_IMAGE_ALL_DEP compiles gem5.opt with GCC. We run a compilation of
# gem5.fast on the Clang compiler to ensure changes are compilable with the
# clang compiler.
rm -rf build
docker run -u $UID:$GID --volume $(pwd):$(pwd) -w $(pwd) --rm \
    "${DOCKER_IMAGE_CLANG_COMPILE}" /usr/bin/env python3 /usr/bin/scons \
    build/ALL/gem5.fast -j4 --no-compress-debug \
    --ignore-style
