# Copyright (c) 2025 The Regents of the University of California
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

"""
This runs simple tests to ensure that running binaries via readfile works.
"""
import os
import re

from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")

readfile_verifier = verifier.MatchRegex(re.compile(r"Readfile test passed!"))


def test_readfile(isa: str, length: str):
    gem5_verify_config(
        name=f"test_readfile_{isa}",
        fixtures=(),
        verifiers=(readfile_verifier,),
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "readfile_tests",
            "configs",
            "ubuntu-run-with-readfile.py",
        ),
        config_args=[
            "--isa",
            isa,
            "--resource-directory",
            resource_path,
        ],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=constants.supported_hosts,
        length=length,
    )


test_readfile(isa="x86", length=constants.long_tag)

test_readfile(isa="riscv", length=constants.long_tag)

test_readfile(isa="arm", length=constants.long_tag)
