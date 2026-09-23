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

# Exercise the complete configuration, not only the underlying Docker image.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
variant=${1:-devcontainer}

test "$(id -un)" = gem5
test "$(id -u)" -ne 0
test -w "$HOME"
sudo -n true

# Repeating setup must preserve the checkout and leave working hook installs.
./.devcontainer/on-create.sh
test -x "$(git rev-parse --git-path hooks/pre-commit)"
test -x "$(git rev-parse --git-path hooks/commit-msg)"
.venv/bin/python - <<'PYCODE'
from importlib.metadata import version
from pathlib import Path
import sys

assert Path(sys.prefix).resolve() == Path(".venv").resolve()
for requirement in Path("requirements.txt").read_text().splitlines():
    if requirement and not requirement.startswith("#"):
        package, expected = requirement.split("==")
        assert version(package) == expected, requirement
PYCODE

for directory in "$PRE_COMMIT_HOME" "$CCACHE_DIR" "$GEM5_RESOURCE_DIR"; do
    test -d "$directory"
    test -w "$directory"
done
ccache --show-stats
# This hook is excluded from file selection but must run with --all-files.
pre-commit run gem5-git-clang-format --all-files
./.devcontainer/build.sh database NULL 2
test -s .devcontainer-cache/compile_commands.json
scons build/NULL/base/bitunion.test.opt -j2
build/NULL/base/bitunion.test.opt
gdb --batch -ex run -ex 'quit $_exitcode' --args /bin/true

case "$variant" in
    devcontainer-demo)
        gem5-release --version
        ;;
    devcontainer-workloads)
        qemu-img --version
        aarch64-linux-gnu-g++ --version
        riscv64-linux-gnu-g++ --version
        x86_64-linux-gnu-g++ --version
        docker info
        ;;
    devcontainer)
        if command -v gem5-release; then
            echo "The development image should not bundle a release." >&2
            exit 1
        fi
        ;;
    *) echo "Unknown devcontainer variant: $variant" >&2; exit 1 ;;
esac
