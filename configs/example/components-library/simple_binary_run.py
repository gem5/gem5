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
A run script for a very simple Syscall-Execution running a hello world program.
The system has no cache heirarchy.

This is as simple a setup as gem5 will allow.
"""

import m5
from m5.objects import Root

import os
import sys

# This is a lame hack to get the imports working correctly.
# TODO: This needs fixed.
sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.pardir,
        os.pardir,
        os.pardir,
    )
)

from components_library.boards.simple_board import SimpleBoard
from components_library.cachehierarchies.classic.no_cache import NoCache
from components_library.memory.single_channel import SingleChannelDDR3_1600
from components_library.processors.simple_processor import SimpleProcessor
from components_library.processors.cpu_types import CPUTypes
from components_library.runtime import (
    get_runtime_coherence_protocol,
    get_runtime_isa,
)

# Setup the system.
cache_hierarchy = NoCache()

memory = SingleChannelDDR3_1600()

processor = SimpleProcessor(cpu_type=CPUTypes.ATOMIC, num_cores=1)

motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

motherboard.connect_things()

# Set the workload
thispath = os.path.dirname(os.path.realpath(__file__))
binary = os.path.join(
    thispath, "../../../tests/test-progs/hello/bin/x86/linux/hello"
)
motherboard.set_workload(binary)


# Run the simulation.
print("Running with ISA: {}.".format(get_runtime_isa().name))
print(
    "Running with protocol: {}.".format(get_runtime_coherence_protocol().name)
)
print()

root = Root(full_system=False, system=motherboard)

m5.instantiate()

exit_event = m5.simulate()
print(
    "Exiting @ tick {} because {}.".format(m5.curTick(), exit_event.getCause())
)
