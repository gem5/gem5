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
A run script for running the parsec benchmark suite in gem5.

On the first run of this script it will download the disk image for parsec
and Linux kernel version 5.4.

The disk image is about 8 GB so this can take 10-15 minutes on a fast
connection.
"""

import m5
import m5.ticks
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

from components_library.boards.x86_board import X86Board
from components_library.cachehierarchies.classic.\
    private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from components_library.memory.ddr3_1600_8x8 import DDR3_1600_8x8
from components_library.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from components_library.processors.cpu_types import CPUTypes
from components_library.runtime import (
    get_runtime_isa,
    get_runtime_coherence_protocol,
)

import subprocess
import gzip
import time
import shutil
import time


# Setup the cachie hierarchy.
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="256kB",
)

# Setup the memory system.
# Warning!!! This must be kept at 3GB for now. X86Motherboard does not support
# anything else right now!
memory = DDR3_1600_8x8(size="3GB")


# The processor. In this case we use the special "SwitchableProcessor" which
# allows us to switch between different SimpleProcessors.
#
# WARNING: This is known buggy. While it works starting with Atomic and
# switching to Timing. The reverse is not true. KVM is not yet functional.

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.TIMING,
    num_cores=4,
)

# Setup the board.
motherboard = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    exit_on_work_items=True,
)

motherboard.connect_things()


# Download the linux kernel and parsec disk image needed to run the
# simuluation.
thispath = os.path.dirname(os.path.realpath(__file__))

kernel_url = (
    "http://dist.gem5.org/dist/v21-0/kernels/x86/static/vmlinux-5.4.49"
)
kernel_path = os.path.join(thispath, "vmlinux-5.4.49")
if not os.path.exists(kernel_path):
    subprocess.run(["wget", "-P", thispath, kernel_url])

parsec_img_url = (
    "http://dist.gem5.org/dist/v21-0/images/x86/ubuntu-18-04/parsec.img.gz"
)
parsec_img_path_gz = os.path.join(thispath, "parsec.img.gz")
parsec_img_path = os.path.join(thispath, "parsec.img")

if not os.path.exists(parsec_img_path):
    subprocess.run(["wget", "-P", thispath, parsec_img_url])
    with gzip.open(parsec_img_path_gz, "rb") as f:
        with open(parsec_img_path, "wb") as o:
            shutil.copyfileobj(f, o)


# The command to run. In this case the blackscholes app with the simsmall
# workload.
command = "cd /home/gem5/parsec-benchmark\n"
command += "source env.sh\n"
command += "parsecmgmt -a run -p blackscholes "
command += "-c gcc-hooks -i simsmall -n {}\n".format(processor.get_num_cores())
command += "sleep 5 \n"
command += "m5 exit \n"

motherboard.set_workload(
    kernel=kernel_path, disk_image=parsec_img_path, command=command
)

print("Running with ISA: " + get_runtime_isa().name)
print("Running with protocol: " + get_runtime_coherence_protocol().name)
print()

root = Root(full_system=True, system=motherboard)

m5.instantiate()

globalStart = time.time()
print("Beginning the simulation")

start_tick = m5.curTick()
end_tick = m5.curTick()

m5.stats.reset()

exit_event = m5.simulate()

if exit_event.getCause() == "workbegin":
    print("Done booting Linux")
    # Reached the start of ROI.
    # The start of the ROI is marked by an m5_work_begin() call.
    print("Resetting stats at the start of ROI!")
    m5.stats.reset()
    start_tick = m5.curTick()

    # Switch to the Timing Processor.
    motherboard.get_processor().switch()
else:
    print("Unexpected termination of simulation!")
    print()

    m5.stats.dump()
    end_tick = m5.curTick()

    m5.stats.reset()
    print("Performance statistics:")
    print("Simulated time: {}s".format((end_tick - start_tick) / 1e12))
    print("Ran a total of", m5.curTick() / 1e12, "simulated seconds")
    print(
        "Total wallclock time: {}s, {} min".format(
            (time.time() - globalStart, (time.time() - globalStart) / 60)
        )
    )
    exit()

# Simulate the ROI.
exit_event = m5.simulate()

if exit_event.getCause() == "workend":
    # Reached the end of ROI
    # The end of the ROI is marked by an m5_work_end() call.
    print("Dumping stats at the end of the ROI!")
    m5.stats.dump()
    end_tick = m5.curTick()

    m5.stats.reset()

    # Switch back to the Atomic Processor
    motherboard.get_processor().switch()
else:
    print("Unexpected termination of simulation!")
    print()
    m5.stats.dump()
    end_tick = m5.curTick()

    m5.stats.reset()
    print("Performance statistics:")
    print("Simulated time: {}s".format((end_tick - start_tick) / 1e12))
    print("Ran a total of", m5.curTick() / 1e12, "simulated seconds")
    print(
        "Total wallclock time: {}s, {} min".format(
            time.time() - globalStart, (time.time() - globalStart) / 60
        )
    )
    exit()

# Simulate the remaning part of the benchmark
# Run the rest of the workload until m5 exit

exit_event = m5.simulate()

print("Done running the simulation")
print()
print("Performance statistics:")

print("Simulated time in ROI: {}s".format((end_tick - start_tick) / 1e12))
print("Ran a total of {} simulated seconds".format(m5.curTick() / 1e12))
print(
    "Total wallclock time: {}s, {} min".format(
        time.time() - globalStart, (time.time() - globalStart) / 60
    )
)
