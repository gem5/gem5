# Copyright (c) 2026 The Board of Trustees of the Leland Stanford
# Junior University
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

"""Tests for X86PinCPU.

These require gem5 to be built with USE_PIN=y and require Pin to be
installed at ext/pin.
"""

from testlib import *

workloads = ("Bubblesort", "FloatMM")

base_path = joinpath(config.bin_path, "pincpu_tests")
base_url = config.resource_url + "/test-progs/cpu-tests/bin/x86"

for workload in workloads:
    workload_binary = DownloadedProgram(
        f"{base_url}/{workload}", base_path, workload
    )
    binary = joinpath(workload_binary.path, workload)
    ref_path = joinpath(getcwd(), "ref", workload)

    # Run the workload.
    gem5_verify_config(
        name=f"pincpu_se_{workload}",
        verifiers=(verifier.MatchStdout(ref_path),),
        fixtures=(workload_binary,),
        config=joinpath(getcwd(), "run.py"),
        config_args=[binary],
        valid_isas=(constants.x86_tag,),
        valid_hosts=(constants.host_x86_64_tag,),
        length=constants.quick_tag,
        uses_pin=True,
    )

    # Run the workload, but interrupt it to take a checkpoint.
    checkpoint_ref_path = joinpath(getcwd(), "ref", f"{workload}_checkpoint")
    gem5_verify_config(
        name=f"pincpu_checkpoint_{workload}",
        verifiers=(verifier.MatchStdout(checkpoint_ref_path),),
        fixtures=(workload_binary,),
        config=joinpath(getcwd(), "run.py"),
        config_args=[binary, "--checkpoint-at", "20000"],
        valid_isas=(constants.x86_tag,),
        valid_hosts=(constants.host_x86_64_tag,),
        length=constants.quick_tag,
        uses_pin=True,
    )
