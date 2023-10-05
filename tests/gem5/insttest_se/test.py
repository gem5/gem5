# Copyright (c) 2020 The Regents of the University of California
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

"""
Test file for the insttest binary running on the SPARC ISA
"""
from testlib import *

test_progs = {constants.sparc_tag: ("sparc-insttest",)}

cpu_types = {constants.sparc_tag: ("atomic", "timing")}

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")

for isa in test_progs:
    for binary in test_progs[isa]:
        ref_path = joinpath(getcwd(), "ref")
        verifiers = (
            verifier.MatchStdoutNoPerf(joinpath(ref_path, "simout.txt")),
        )

        for cpu in cpu_types[isa]:
            gem5_verify_config(
                name="test-" + binary + "-" + cpu,
                fixtures=(),
                verifiers=verifiers,
                config=joinpath(
                    config.base_dir,
                    "tests",
                    "gem5",
                    "insttest_se",
                    "configs",
                    "simple_binary_run.py",
                ),
                config_args=[
                    binary,
                    cpu,
                    "--resource-directory",
                    resource_path,
                    "sparc",
                ],
                valid_isas=(constants.all_compiled_tag,),
                length=constants.long_tag,
            )
