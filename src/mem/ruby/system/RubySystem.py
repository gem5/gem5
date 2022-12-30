# Copyright (c) 2009 Advanced Micro Devices, Inc.
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

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject
from m5.objects.SimpleMemory import *


class RubySystem(ClockedObject):
    type = "RubySystem"
    cxx_header = "mem/ruby/system/RubySystem.hh"
    cxx_class = "gem5::ruby::RubySystem"

    randomization = Param.Bool(
        False,
        "insert random delays on message enqueue times (if True, all message \
         buffers are enforced to have randomization; otherwise, a message \
         buffer set its own flag to enable/disable randomization)",
    )
    block_size_bytes = Param.UInt32(
        64, "default cache block size; must be a power of two"
    )
    memory_size_bits = Param.UInt32(
        64, "number of bits that a memory address requires"
    )

    phys_mem = Param.SimpleMemory(NULL, "")
    system = Param.System(Parent.any, "system object")

    access_backing_store = Param.Bool(
        False,
        "Use phys_mem as the functional \
        store and only use ruby for timing.",
    )

    # Profiler related configuration variables
    hot_lines = Param.Bool(False, "")
    all_instructions = Param.Bool(False, "")
    num_of_sequencers = Param.Int("")
    number_of_virtual_networks = Param.Unsigned("")
