# Copyright (c) 2026 Barcelona Supercomputing Center
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
Runs an RVV program on an O3 CPU whose memory instructions grow and shrink
their LSQ occupancy, under each overflow mode, and checks that it finishes
with the expected output.

The drain tests drain the CPU repeatedly while the vector loop runs, so it
drains with resized entries and waiting accesses in flight. Each drain
runs the checks in ``drainSanityCheck()``, which assert that the occupied
entries are back to zero and that no access is left waiting to replay.

The switch tests switch to the Timing CPU and back during the vector loop.

The config fails if the program finishes before the drains or the switches
are done.
"""

import re

from testlib import *

resource = "rvv-memcpy"

# The drain tests start just before the vector loop. The switch tests go to
# the Timing CPU during the loop, for 50000 ticks, so the O3 CPU handles
# growing accesses on both sides of the switch.
variants = {
    "": [],
    "-drain": ["--drain-from", "45850000"],
    "-switch": ["--switch-at", "45950000", "--timing-ticks", "50000"],
}

# Stats showing that each mode handled accesses that grew past their
# reservation.
mode_stats = {
    "Bound": "entryClamps",
    "Overcommit": "entryOvercommits",
    "Replay": "entryReplays",
}

config_path = joinpath(
    config.base_dir,
    "tests",
    "gem5",
    "o3_lsq_occupancy",
    "configs",
    "lsq_occupancy_se.py",
)

for mode, stat in mode_stats.items():
    # A suite can only have one verifier of each class.
    verifiers = (
        verifier.MatchRegex(
            re.compile(rf"^.*{resource}(-\d+\.\d+\.\d+)?: pass$")
        ),
        verifier.MatchFileRegex(
            re.compile(rf"^\S+\.lsq0\.{stat}\s+[1-9]"), ["stats.txt"]
        ),
    )

    for suffix, variant_args in variants.items():
        gem5_verify_config(
            name=f"test-lsq-occupancy-{resource}-{mode}{suffix}",
            fixtures=(),
            verifiers=verifiers,
            config=config_path,
            config_args=[resource, "--mode", mode] + variant_args,
            valid_isas=(constants.all_compiled_tag,),
            length=constants.quick_tag,
        )
