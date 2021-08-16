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
This example runs a simple boot exit test.

Characteristics
---------------

* Runs exclusively on the X86 ISA with the MESI_TWO_LEVEL coherence protocol.
"""

import m5
from m5.objects import Root

import sys
import os

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

from components_library.runtime import (
    get_runtime_coherence_protocol,
    get_runtime_isa,
)
from components_library.boards.x86_board import X86Board
from components_library.memory.single_channel import SingleChannelDDR3_1600
from components_library.processors.simple_processor import SimpleProcessor
from components_library.processors.cpu_types import CPUTypes
from components_library.isas import ISA
from components_library.coherence_protocol import CoherenceProtocol
from components_library.resources.resource import Resource

import os

# Run a check to ensure the right version of gem5 is being used.
if (
    get_runtime_coherence_protocol() != CoherenceProtocol.MESI_TWO_LEVEL
    or get_runtime_isa() != ISA.X86
):
    raise EnvironmentError(
        "The boot-exit-disk_run.py should be run with X86_MESI_Two_Level."
    )

from components_library.cachehierarchies.\
    ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)

# Setup the cache heirarchy to be MESI_Two_Level.
cache_hierarchy = MESITwoLevelCacheHierarchy(
    l1d_size="32kB",
    l1d_assoc=8,
    l1i_size="32kB",
    l1i_assoc=8,
    l2_size="256kB",
    l2_assoc=16,
    num_l2_banks=1,
)

# Setup the system memory.
# Warning: This must be kept at 3GB for now. X86Motherboard does not support
# anything else right now!
memory = SingleChannelDDR3_1600(size="3GB")

# Setup a single core Timing Processor.
processor = SimpleProcessor(cpu_type=CPUTypes.TIMING, num_cores=1)

# Setup the motherboard.
motherboard = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    exit_on_work_items=True,
)

motherboard.connect_things()

# Set the Full System workload.
motherboard.set_workload(
    kernel=Resource("x86-linux-kernel-5.4.49"),
    disk_image=Resource("x86-boot-exit"), command="m5 exit \n"
)


# Begin running of the simulation. This will exit once the Linux system boot
# is complete.
print("Running with ISA: " + get_runtime_isa().name)
print("Running with protocol: " + get_runtime_coherence_protocol().name)
print()

root = Root(full_system=True, system=motherboard)

m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print(
    "Exiting @ tick {} because {}.".format(m5.curTick(), exit_event.getCause())
)
