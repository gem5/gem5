# Copyright (c) 2026 Nathanon Chirattithiphan
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

from pathlib import Path

from testlib import (
    constants,
    gem5_verify_config,
    verifier,
)

config = Path(__file__).resolve().parent / "configs" / "mwait.py"

cases = (
    ("atomic", "unarmed"),
    ("o3", "unarmed"),
    ("o3", "armed"),
    ("o3", "wrong-monitor"),
    ("o3", "wrong-mwait"),
)

for cpu, case in cases:
    verifiers = ()
    if case == "armed":
        # O3 accounts for its time suspended when the snoop reactivates it.
        verifiers = (
            verifier.MatchFileRegex(
                r"^system\.cpu\.quiesceCycles\s+[1-9]\d*\s",
                (constants.gem5_simulation_stats,),
            ),
        )
    gem5_verify_config(
        name=f"x86-mwait-{cpu}-{case}",
        config=str(config),
        config_args=[cpu, case],
        verifiers=verifiers,
        valid_isas=(constants.x86_tag, constants.all_compiled_tag),
        valid_hosts=(constants.host_x86_64_tag,),
        valid_variants=(constants.opt_tag,),
        length=constants.quick_tag,
    )
