# Copyright (c) 2021 The Regents of the University of California
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
This file contains functions to extract gem5 runtime information.
"""

from m5.defines import buildEnv

from .isas import ISA
from .coherence_protocol import CoherenceProtocol


def get_runtime_isa() -> ISA:
    """Gets the target ISA.
    This can be inferred at runtime.

    :returns: The target ISA.
    """
    isa_map = {
        "sparc": ISA.SPARC,
        "mips": ISA.MIPS,
        "null": ISA.NULL,
        "arm": ISA.ARM,
        "x86": ISA.X86,
        "power": ISA.POWER,
        "riscv": ISA.RISCV,
    }

    isa_str = str(buildEnv["TARGET_ISA"]).lower()
    if isa_str not in isa_map.keys():
        raise NotImplementedError(
            "ISA '" + buildEnv["TARGET_ISA"] + "' not recognized."
        )

    return isa_map[isa_str]


def get_runtime_coherence_protocol() -> CoherenceProtocol:
    """Gets the cache coherence protocol.
    This can be inferred at runtime.

    :returns: The cache coherence protocol.
    """
    protocol_map = {
        "mi_example": CoherenceProtocol.MI_EXAMPLE,
        "moesi_hammer": CoherenceProtocol.ARM_MOESI_HAMMER,
        "garnet_standalone": CoherenceProtocol.GARNET_STANDALONE,
        "moesi_cmp_token": CoherenceProtocol.MOESI_CMP_TOKEN,
        "mesi_two_level": CoherenceProtocol.MESI_TWO_LEVEL,
        "moesi_amd_base": CoherenceProtocol.MOESI_AMD_BASE,
        "mesi_three_level_htm": CoherenceProtocol.MESI_THREE_LEVEL_HTM,
        "mesi_three_level": CoherenceProtocol.MESI_THREE_LEVEL,
        "gpu_viper": CoherenceProtocol.GPU_VIPER,
        "chi": CoherenceProtocol.CHI,
    }

    protocol_str = str(buildEnv["PROTOCOL"]).lower()
    if protocol_str not in protocol_map.keys():
        raise NotImplementedError(
            "Protocol '" + buildEnv["PROTOCOL"] + "' not recognized."
        )

    return protocol_map[protocol_str]
