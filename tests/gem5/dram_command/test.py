# Copyright (c) 2026 CkingX256
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
import sys
from pathlib import Path

from testlib import *


class CheckDramCommands(verifier.Verifier):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy
        self.checker = joinpath(getcwd(), "check_trace.py")

    def test(self, params):
        directory = params.fixtures[constants.tempdir_fixture_name].path
        binary = params.fixtures[constants.gem5_binary_fixture_name].path
        disabled = Path(directory) / "disabled"
        disabled.mkdir()
        with (disabled / "console.log").open("w") as console:
            subprocess.run(
                [
                    binary,
                    f"--outdir={disabled}",
                    joinpath(
                        config.base_dir, "configs", "dram", "command_trace.py"
                    ),
                    "--page-policy",
                    self.policy,
                ],
                check=True,
                stdout=console,
                stderr=subprocess.STDOUT,
                timeout=60,
            )
        assert not (disabled / "dram-commands.csv").exists()
        subprocess.run(
            [
                sys.executable,
                self.checker,
                joinpath(directory, "dram-commands.csv"),
                "--policy",
                self.policy,
                "--compare-stats",
                joinpath(directory, "stats.txt"),
                str(disabled / "stats.txt"),
            ],
            check=True,
            timeout=60,
        )


for policy in ("open", "close"):
    gem5_verify_config(
        name=f"dram-command-observation-{policy}",
        fixtures=(),
        verifiers=(CheckDramCommands(policy),),
        config=joinpath(
            config.base_dir, "configs", "dram", "command_trace.py"
        ),
        config_args=["--trace", "--page-policy", policy],
        valid_isas=(constants.null_tag, constants.all_compiled_tag),
        valid_hosts=constants.supported_hosts,
        length=constants.quick_tag,
    )
