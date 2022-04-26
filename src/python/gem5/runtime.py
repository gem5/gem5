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
from m5.objects import RubyProtocols

from .isas import ISA, get_isa_from_str, get_isas_str_set
from typing import Set


def get_supported_isas() -> Set[ISA]:
    """
    Returns the set of all the ISAs compiled into the current binary.
    """
    supported_isas = set()

    if "TARGET_ISA" in buildEnv.keys():
        supported_isas.add(get_isa_from_str(buildEnv["TARGET_ISA"]))

    for key in get_isas_str_set():
        if buildEnv.get(f"USE_{key.upper()}_ISA", False):
            supported_isas.add(get_isa_from_str(key))

    return supported_isas

def get_supported_ruby_protocols() -> Set[RubyProtocols]:
    """Returns the Ruby protocols supported in this gem5 binary"""
    return set(
        RubyProtocols(protocol) for protocol in buildEnv["BUILD_PROTOCOLS"]
    )

def get_runtime_isa() -> ISA:
    """
    Returns a single target ISA at runtime.

    This determined via the "TARGET_ISA" parameter, which is set at
    compilation. If not set, but only one ISA is compiled, we assume it's the
    one ISA. If neither the "TARGET_ISA" parameter is set and there are
    multiple ISA targets, an exception is thrown.

    **WARNING**: This function is deprecated and may be removed in future
    versions of gem5. This function should not be relied upon to run gem5
    simulations.

    :returns: The target ISA.
    """

    warn(
        "The `get_runtime_isa` function is deprecated. Please migrate away "
        "from using this function."
    )

    if "TARGET_ISA" in buildEnv.keys():
        return get_isa_from_str(buildEnv["TARGET_ISA"])

    supported_isas = get_supported_isas()

    if len(supported_isas) == 1:
        return next(iter(supported_isas))

    raise Exception(
        "Cannot determine the the runtime ISA. Either the "
        "'TARGET_ISA' parameter must be set or the binary only "
        "compiled to one ISA."
    )

    return main_isa
