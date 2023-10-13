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

from ..boards.mem_mode import MemMode

from enum import Enum
from typing import Set
import os


class CPUTypes(Enum):
    ATOMIC = "atomic"
    KVM = "kvm"
    O3 = "o3"
    TIMING = "timing"
    MINOR = "minor"


def get_cpu_types_str_set() -> Set[str]:
    """
    Returns a set of all the CPU types as strings.
    """
    return {cpu_type.value for cpu_type in CPUTypes}


def get_cpu_type_from_str(input: str) -> CPUTypes:
    """
    Will return the correct enum given the input string. This is matched on
    the enum's value. E.g., "kvm" will return ISA.KVM. Throws an exception if
    the input string is invalid.

    `get_cpu_types_str_set()` can be used to determine the valid strings.

    This is for parsing text inputs that specify CPU Type targets.

    :param input: The CPU Type to return, as a string. Case-insensitive.
    """
    for cpu_type in CPUTypes:
        if input.lower() == cpu_type.value:
            return cpu_type

    valid_cpu_types_list_str = ""
    for cpu_type_str in get_cpu_types_str_set():
        valid_cpu_types_list_str += f"{os.linesep}{cpu_type_str}"

    raise Exception(
        f"CPU type '{input}' does not correspond to a known CPU type. "
        f"Known CPU Types:{valid_cpu_types_list_str}"
    )


def get_mem_mode(input: CPUTypes) -> MemMode:
    """
    Returns the correct memory mode to be set for a given CPUType.

    :param input: The CPUType to check.
    """

    cpu_mem_mode_map = {
        CPUTypes.TIMING: MemMode.TIMING,
        CPUTypes.O3: MemMode.TIMING,
        CPUTypes.MINOR: MemMode.TIMING,
        CPUTypes.KVM: MemMode.ATOMIC_NONCACHING,
        CPUTypes.ATOMIC: MemMode.ATOMIC,
    }

    return cpu_mem_mode_map[input]
