# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.objects.AbstractMemory import AbstractMemory
from m5.params import *
from m5.proxy import *


class ScratchpadMemory(AbstractMemory):
    type = "ScratchpadMemory"
    cxx_header = "salam/scratchpad_memory.hh"

    port = ResponsePort("Generic response port")
    spm_ports = VectorResponsePort(
        "Response ports for private acclerator SPM accesses"
    )
    latency = Param.Latency("2ns", "Request to response latency")
    latency_var = Param.Latency("0ns", "Request to response latency variance")
    ready_mode = Param.Bool(False, "Use ready mode for scratchpad memory")
    read_on_invalid = Param.Bool(
        False,
        "Enable reads on invalid memory segments when ready mode is used",
    )
    write_on_valid = Param.Bool(
        True, "Enable writes on valid memory sectors when ready mode is used"
    )
    reset_on_scratchpad_read = Param.Bool(
        True, "Reset ready bit on private scratchpad memory read"
    )
    bandwidth = Param.MemoryBandwidth(
        "12GiB/s", "Combined read and write bandwidth per port"
    )
