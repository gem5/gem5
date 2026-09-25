#!/bin/bash

# Copyright (c) 2026 The Regents of The University of California
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

# Build the selected configuration and expose its actual compiler commands.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

action=${1:-build}
isa=${2:-ALL}
jobs=${3:-2}
if [[ ! "$isa" =~ ^[A-Za-z0-9_]+$ || ! -f "build_opts/$isa" ]]; then
    echo "Unknown build configuration: $isa" >&2
    exit 1
fi
if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
    echo "Build jobs must be a positive integer." >&2
    exit 1
fi

targets=("build/$isa/compile_commands.json")
case "$action" in
    build) targets+=("build/$isa/gem5.opt") ;;
    database) ;;
    *) echo "Expected build or database." >&2; exit 1 ;;
esac
scons "${targets[@]}" -j"$jobs"
mkdir -p .devcontainer-cache
ln -sfn "../build/$isa/compile_commands.json" \
    .devcontainer-cache/compile_commands.json
