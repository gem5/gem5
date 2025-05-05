# Copyright (c) 2022 The Regents of the University of California
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

from testlib import *

isa_map = {
    "sparc": constants.sparc_tag,
    "mips": constants.mips_tag,
    "null": constants.null_tag,
    "arm": constants.arm_tag,
    "x86": constants.vega_x86_tag,
    "power": constants.power_tag,
    "riscv": constants.riscv_tag,
}

length_map = {
    "sparc": constants.long_tag,
    "mips": constants.long_tag,
    "null": constants.long_tag,
    "arm": constants.very_long_tag,
    "x86": constants.very_long_tag,
    "power": constants.long_tag,
    "riscv": constants.long_tag,
}

for isa in isa_map.keys():
    if isa in ("x86", "arm"):
        # We only do these checks for X86 and ARM to save compiling
        # other ISAs.
        gem5_verify_config(
            name=f"requires-isa-{isa}",
            verifiers=(),
            fixtures=(),
            config=joinpath(
                config.base_dir,
                "tests",
                "gem5",
                "stdlib",
                "configs",
                "requires_check.py",
            ),
            config_args=["-i", isa],
            valid_isas=(constants.all_compiled_tag,),
            length=length_map[isa],
        )

    if isa != "null":
        gem5_verify_config(
            name=f"requires-isa-{isa}-with-all-compiled",
            verifiers=(),
            fixtures=(),
            config=joinpath(
                config.base_dir,
                "tests",
                "gem5",
                "stdlib",
                "configs",
                "requires_check.py",
            ),
            config_args=["-i", isa],
            valid_isas=(constants.all_compiled_tag,),
            length=constants.quick_tag,
        )
