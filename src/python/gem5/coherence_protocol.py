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
Specifies the coherence protocol enum
"""

import os
from enum import Enum


class CoherenceProtocol(Enum):
    NULL = "no protocol (classic only)"
    MESI_THREE_LEVEL = "MESI_Three_Level"
    MESI_THREE_LEVEL_HTM = "MESI_Three_Level_HTM"
    AMD_MOESI_HAMMER = "AMD_MOESI_hammer"
    GARNET_STANDALONE = "Garnet_standalone"
    MESI_TWO_LEVEL = "MESI_Two_Level"
    MOESI_CMP_DIRECTORY = "MOESI_CMP_directory"
    MOESI_CMP_TOKEN = "MOESI_CMP_token"
    MOESI_AMD_BASE = "MOESI_AMD_Base"
    MI_EXAMPLE = "MI_example"
    GPU_VIPER = "GPU_VIPER"
    CHI = "CHI"
    MSI = "MSI"


def get_protocols_str_set():
    return {protocol.value for protocol in CoherenceProtocol}


def get_protocol_from_str(protocol_str: str) -> CoherenceProtocol:
    """
    Will return the correct enum given the input string. This is matched on
    the enum's value. E.g., "CHI" will return CoherenceProtocol.CHI. Throws
    an exception if the input string is invalid.

    ``get_protocols_str_set()`` can be used to determine the valid strings.

    This is for parsing text inputs that specify protocol targets.

    :param input: The protocol to return, as a string. Case-insensitive.
    """

    for protocol in CoherenceProtocol:
        if protocol.value.lower() == protocol_str.lower():
            return protocol

    valid_protocols_str_list = ""
    for isa_str in get_protocols_str_set():
        valid_protocols_str_list += f"{os.linesep}{isa_str}"

    raise Exception(
        f"Value '{input}' does not correspond to a known ISA. Known protocols:"
        f"{valid_protocols_str_list}"
    )
