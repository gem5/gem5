# Copyright (c) 2026 The Regents of The University of California
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

import subprocess
from pathlib import Path

import pytest


@pytest.fixture
def gem5(request, tmp_path):
    """Run a case with its explicitly prebuilt simulator."""
    root = request.config.rootpath
    build = request.node.get_closest_marker("gem5").kwargs["build"]
    binary = root / build

    def run(config, *args):
        if not binary.is_file():
            pytest.fail(f"Missing {build}. Build it with 'scons {build}'.")
        if not Path(config).is_absolute():
            config = root / config
        try:
            result = subprocess.run(
                [str(binary), "-d", str(tmp_path), str(config), *args],
                cwd=root,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
            )
        except OSError as error:
            pytest.fail(f"Could not execute {binary}: {error}", pytrace=False)
        if result.returncode != 0:
            pytest.fail(
                f"gem5 exited with status {result.returncode}. "
                f"Output directory: {tmp_path}\n"
                f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                pytrace=False,
            )
        return result

    return run
