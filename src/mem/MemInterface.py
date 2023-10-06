# Copyright (c) 2012-2020 ARM Limited
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
# Copyright (c) 2013 Amin Farmahini-Farahani
# Copyright (c) 2015 University of Kaiserslautern
# Copyright (c) 2015 The University of Bologna
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
from m5.objects.AbstractMemory import AbstractMemory
from m5.params import *
from m5.proxy import *


# Enum for the address mapping. With Ch, Ra, Ba, Ro and Co denoting
# channel, rank, bank, row and column, respectively, and going from
# MSB to LSB.  Available are RoRaBaChCo and RoRaBaCoCh, that are
# suitable for an open-page policy, optimising for sequential accesses
# hitting in the open row. For a closed-page policy, RoCoRaBaCh
# maximises parallelism.
class AddrMap(Enum):
    vals = ["RoRaBaChCo", "RoRaBaCoCh", "RoCoRaBaCh"]


class MemInterface(AbstractMemory):
    type = "MemInterface"
    abstract = True
    cxx_header = "mem/mem_interface.hh"
    cxx_class = "gem5::memory::MemInterface"

    # Allow the interface to set required controller buffer sizes
    # each entry corresponds to a burst for the specific memory channel
    # configuration (e.g. x32 with burst length 8 is 32 bytes) and not
    # the cacheline size or request/packet size
    write_buffer_size = Param.Unsigned(64, "Number of write queue entries")
    read_buffer_size = Param.Unsigned(32, "Number of read queue entries")

    # scheduler, address map
    addr_mapping = Param.AddrMap("RoRaBaCoCh", "Address mapping policy")

    # size of memory device in Bytes
    device_size = Param.MemorySize("Size of memory device")
    # the physical organisation of the memory
    device_bus_width = Param.Unsigned(
        "data bus width in bits for each memory device/chip",
    )
    burst_length = Param.Unsigned("Burst lenght (BL) in beats")
    device_rowbuffer_size = Param.MemorySize(
        "Page (row buffer) size per device/chip",
    )
    devices_per_rank = Param.Unsigned("Number of devices/chips per rank")
    ranks_per_channel = Param.Unsigned("Number of ranks per channel")
    banks_per_rank = Param.Unsigned("Number of banks per rank")

    # timing behaviour and constraints - all in nanoseconds

    # the base clock period of the memory
    tCK = Param.Latency("Clock period")

    # time to complete a burst transfer, typically the burst length
    # divided by two due to the DDR bus, but by making it a parameter
    # it is easier to also evaluate SDR memories like WideIO and new
    # interfaces, emerging technologies.
    # This parameter has to account for burst length.
    # Read/Write requests with data size larger than one full burst are broken
    # down into multiple requests in the controller
    tBURST = Param.Latency(
        "Burst duration (typically burst length / 2 cycles)",
    )

    # write-to-read, same rank turnaround penalty
    tWTR = Param.Latency("Write to read, same rank switching time")

    # read-to-write, same rank turnaround penalty
    tRTW = Param.Latency("Read to write, same rank switching time")

    # rank-to-rank bus delay penalty
    # this does not correlate to a memory timing parameter and encompasses:
    # 1) RD-to-RD, 2) WR-to-WR, 3) RD-to-WR, and 4) WR-to-RD
    # different rank bus delay
    tCS = Param.Latency("Rank to rank switching time")
