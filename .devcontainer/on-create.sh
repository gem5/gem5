#!/bin/bash

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

# Prepare tools and caches during creation and Codespaces prebuild updates.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

# Making the downloaded repository safe as the owner might differ for .devcontainer env.
git config --global --add safe.directory /workspaces/gem5

# Keep caches with the persistent workspace, including for local containers.
mkdir -p "${PRE_COMMIT_HOME}" "${CCACHE_DIR}" "${GEM5_RESOURCE_DIR}"

# Reuse distro modules such as SCons and pydot while installing the exact
# development-tool versions requested by this checkout into a writable venv.
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
export PATH="${PWD}/.venv/bin:${PATH}"

# Prepare hook environments now instead of downloading them on first commit.
pre-commit install --install-hooks -t pre-commit -t commit-msg
