# Copyright (c) 2020 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
from m5.objects.MemCtrl import MemCtrl
from m5.objects.MemInterface import MemInterface
from m5.objects.DRAMInterface import AddrMap

# The following interface aims to model byte-addressable NVM
# The most important system-level performance effects of a NVM
# are modeled without getting into too much detail of the media itself.
class NVMInterface(MemInterface):
    type = "NVMInterface"
    cxx_header = "mem/nvm_interface.hh"
    cxx_class = "gem5::memory::NVMInterface"

    # NVM DIMM could have write buffer to offload writes
    # define buffer depth, which will limit the number of pending writes
    max_pending_writes = Param.Unsigned("1", "Max pending write commands")

    # NVM DIMM could have buffer to offload read commands
    # define buffer depth, which will limit the number of pending reads
    max_pending_reads = Param.Unsigned("1", "Max pending read commands")

    # timing behaviour and constraints - all in nanoseconds

    # define average latency for NVM media.  Latency defined uniquely
    # for read and writes as the media is typically not symmetric
    tREAD = Param.Latency("100ns", "Average NVM read latency")
    tWRITE = Param.Latency("200ns", "Average NVM write latency")
    tSEND = Param.Latency("15ns", "Access latency")

    two_cycle_rdwr = Param.Bool(
        False, "Two cycles required to send read and write commands"
    )

    def controller(self):
        """
        Instantiate the memory controller and bind it to
        the current interface.
        """
        controller = MemCtrl()
        controller.dram = self
        return controller


# NVM delays and device architecture defined to mimic PCM like memory.
# Can be configured with DDR4_2400 sharing the channel
class NVM_2400_1x64(NVMInterface):
    write_buffer_size = 128
    read_buffer_size = 64

    max_pending_writes = 128
    max_pending_reads = 64

    device_rowbuffer_size = "256B"

    # 8X capacity compared to DDR4 x4 DIMM with 8Gb devices
    device_size = "512GiB"
    # Mimic 64-bit media agnostic DIMM interface
    device_bus_width = 64
    devices_per_rank = 1
    ranks_per_channel = 1
    banks_per_rank = 16

    burst_length = 8

    two_cycle_rdwr = True

    # 1200 MHz
    tCK = "0.833ns"

    tREAD = "150ns"
    tWRITE = "500ns"
    tSEND = "14.16ns"
    tBURST = "3.332ns"

    # Default all bus turnaround and rank bus delay to 2 cycles
    # With DDR data bus, clock = 1200 MHz = 1.666 ns
    tWTR = "1.666ns"
    tRTW = "1.666ns"
    tCS = "1.666ns"
