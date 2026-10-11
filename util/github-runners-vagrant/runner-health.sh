#!/bin/bash

# Copyright (c) 2023 The Regents of the University of California
# All rights reserved.
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

# Run as the service user, including Docker access and bind-mount checks.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
: "${HEALTH_IMAGE:=ubuntu:24.04}"
: "${RUNNER_MIN_FREE_GIB:=20}"
RUNNER_RESOURCE_CACHE=${RUNNER_RESOURCE_CACHE-/gem5-resource-cache}
[[ $(id -u) != 0 ]]
timeout 30 docker info >/dev/null
available=$(df -B1 --output=avail . | tail -1)
(( available >= RUNNER_MIN_FREE_GIB * 1024 * 1024 * 1024 ))
inodes=$(df --output=iavail . | tail -1)
(( inodes >= 100000 ))
[[ -w . ]]
probe=$(mktemp -d "$PWD/.runner-health.XXXXXXXX")
trap 'sudo -n rm -rf -- "$probe"' EXIT
printf 'host-to-container\n' > "$probe/input"
args=(--rm --pull=never --network none -v "$probe:/probe")
if [[ -n "$RUNNER_RESOURCE_CACHE" ]]; then
    # A missing mount must never silently become a private local cache.
    mountpoint -q "$RUNNER_RESOURCE_CACHE"
    args+=(-v "$RUNNER_RESOURCE_CACHE:/cache")
fi
# Expand these expressions inside the container, not on the guest.
# shellcheck disable=SC2016
timeout 60 docker run "${args[@]}" "$HEALTH_IMAGE" sh -ec '
    test "$(cat /probe/input)" = host-to-container
    echo container-to-host > /probe/output
    if [ -d /cache ]; then
        file=$(mktemp /cache/.runner-health.XXXXXXXX)
        echo cache-check > "$file"
        test "$(cat "$file")" = cache-check
        rm "$file"
    fi
'
[[ $(cat "$probe/output") == container-to-host ]]
sudo -n rm -rf -- "$probe"
trap - EXIT
echo "Runner health checks passed"
