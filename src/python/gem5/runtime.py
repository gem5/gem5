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
from m5.util import warn
import os

from .isas import ISA, get_isa_from_str, get_isas_str_set
from .coherence_protocol import CoherenceProtocol
from typing import Set
from m5 import options

def get_supported_isas() -> Set[ISA]:
    supported_isas = set()

    # This if-statement is an intermediate step so the stdlib works on the
    # "old style" of determining the available ISA (via the "TARGET_ISA"
    # environment variable) and the "new style" which enables multiple ISAs
    # (via the "USE_X_ISA" environment variables). Once multiple ISAs are fully
    # incorporated, this code may be simplified.
    if "TARGET_ISA" in buildEnv.keys():
        supported_isas.add(get_isa_from_str(buildEnv["TARGET_ISA"]))
    else:
        for key in get_isas_str_set():
            if buildEnv[f"USE_{key.upper()}_ISA"]:
                supported_isas.add(get_isa_from_str(key))
    return supported_isas



def get_runtime_isa() -> ISA:
    """Returns a single target ISA at runtime.
    This is inferred at runtime and is assumed to be the ISA target ISA.
    This is determined one of two ways:

    1. The gem5 binary is compiled to one ISA target.
    2. The user specifies the target ISA via gem5's `--main-isa` parameter.

    **WARNING**: This function is deprecated and may be removed in future
    versions of gem5. This function should not be relied upon to run gem5
    simulations.

    :returns: The target ISA.
    """

    warn("The `get_runtime_isa` function is deprecated. Please migrate away "
         "from using this function.")

    supported = get_supported_isas()
    main_isa_param = options.main_isa

    supported_list_str = ""
    for isa in supported:
        supported_list_str += f"{os.linesep}{isa.name}"


    if not main_isa_param:
        if len(supported) == 1:
            # In this case, where the main_isa_param is not specified, but only
            # one ISA is compiled, we go with the ISA that has been compiled.
            return next(iter(supported))

        # If there are multiple supported ISAs, but no main ISA specified, we
        # raise an exception.
        raise Exception("The gem5 binary is compiled with multiple-ISAs. "
                        "Please specify which you require using the "
                        "`--main-isa` parameter when running gem5. "
                        f"Supported ISAs: {supported_list_str}"
                       )

    assert main_isa_param

    main_isa = get_isa_from_str(main_isa_param)

    if main_isa not in supported:
        # In the case the user has specified an ISA not compiled into
        # the binary.
        raise Exception(f"The ISA specified via the `--main-isa` "
                        f"parameter, '{main_isa_param}', has not been "
                        f"compiled into the binary. ISAs available: "
                        f"{supported_list_str}."
                       )

    return main_isa


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
