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
Specifies the ISA enum
"""

import os
from enum import Enum
from typing import Set


class ISA(Enum):
    """
    The ISA Enums which may be used in the gem5 stdlib to specify ISAs.

    Their value may be used to translate the ISA to strings and compare against
    inputs and environment variables.

    E.g., to check if the X86 ISA is compiled:

    .. code-block::

            if buildEnv[f"USE_{ISA.X86.value}_ISA"]:
        ...

    """

    X86 = "x86"
    RISCV = "riscv"
    ARM = "arm"
    MIPS = "mips"
    POWER = "power"
    SPARC = "sparc"
    NULL = "null"


def get_isas_str_set() -> Set[str]:
    """
    Returns a set of all the ISA as strings.
    """
    return {isa.value for isa in ISA}


def get_isa_from_str(input: str) -> ISA:
    """
    Will return the correct enum given the input string. This is matched on
    the enum's value. E.g., "x86" will return ISA.X86. Throws an exception if
    the input string is invalid.

    ``get_isas_str_set()`` can be used to determine the valid strings.

    This is for parsing text inputs that specify ISA targets.

    :param input: The ISA to return, as a string. Case-insensitive.
    """
    for isa in ISA:
        if input.lower() == isa.value:
            return isa

    valid_isas_str_list = ""
    for isa_str in get_isas_str_set():
        valid_isas_str_list += f"{os.linesep}{isa_str}"

    raise Exception(
        f"Value '{input}' does not correspond to a known ISA. Known ISAs:"
        f"{valid_isas_str_list}"
    )
