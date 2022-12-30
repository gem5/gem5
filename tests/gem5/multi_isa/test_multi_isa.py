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


for isa in isa_map.keys():
    if isa in ("x86", "arm", "riscv"):
        # We only do these checks for X86, ARM, and RISCV to save compiling
        # other ISAs.
        gem5_verify_config(
            name=f"runtime-isa-check_{isa}-compiled-alone",
            verifiers=(),
            fixtures=(),
            config=joinpath(
                config.base_dir,
                "tests",
                "gem5",
                "configs",
                "runtime_isa_check.py",
            ),
            config_args=["-e", isa],
            valid_isas=(isa_map[isa],),
            valid_hosts=constants.supported_hosts,
            length=constants.long_tag,
        )

        gem5_verify_config(
            name=f"supported-isas-check_{isa}-compiled-alone",
            verifiers=(),
            fixtures=(),
            config=joinpath(
                config.base_dir,
                "tests",
                "gem5",
                "configs",
                "supported_isa_check.py",
            ),
            config_args=["-e", isa],
            valid_isas=(isa_map[isa],),
            valid_hosts=constants.supported_hosts,
            length=constants.long_tag,
        )

    if isa != "null":
        # The null isa is not "supported" in a case where other ISAs are
        # present.
        gem5_verify_config(
            name=f"supported-isas-check_{isa}-all-compiled",
            verifiers=(),
            fixtures=(),
            config=joinpath(
                config.base_dir,
                "tests",
                "gem5",
                "configs",
                "supported_isa_check.py",
            ),
            config_args=["-e", isa],
            valid_isas=(constants.all_compiled_tag,),
            valid_hosts=constants.supported_hosts,
            length=constants.quick_tag,
        )
