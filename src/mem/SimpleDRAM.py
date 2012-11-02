# Copyright (c) 2012 ARM Limited
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
#
# Authors: Andreas Hansson
#          Ani Udipi

from m5.params import *
from AbstractMemory import *

# Enum for memory scheduling algorithms, currently First-Come
# First-Served and a First-Row Hit then First-Come First-Served
class MemSched(Enum): vals = ['fcfs', 'frfcfs']

# Enum for the address mapping, currently corresponding to either
# optimising for sequential accesses hitting in the open row, or
# striping across banks.
class AddrMap(Enum): vals = ['openmap', 'closemap']

# Enum for the page policy, either open or close.
class PageManage(Enum): vals = ['open', 'close']

# SimpleDRAM is a single-channel single-ported DRAM controller model
# that aims to model the most important system-level performance
# effects of a DRAM without getting into too much detail of the DRAM
# itself.
class SimpleDRAM(AbstractMemory):
    type = 'SimpleDRAM'
    cxx_header = "mem/simple_dram.hh"

    # single-ported on the system interface side, instantiate with a
    # bus in front of the controller for multiple ports
    port = SlavePort("Slave port")

    # the physical organisation of the DRAM
    lines_per_rowbuffer = Param.Unsigned(64, "Row buffer size in cache lines")
    ranks_per_channel = Param.Unsigned(2, "Number of ranks per channel")
    banks_per_rank = Param.Unsigned(8, "Number of banks per rank")

    # the basic configuration of the controller architecture
    write_buffer_size = Param.Unsigned(32, "Number of read queue entries")
    read_buffer_size = Param.Unsigned(32, "Number of write queue entries")

    # threshold in percent for when to trigger writes and start
    # emptying the write buffer as it starts to get full
    write_thresh_perc = Param.Percent(70, "Threshold to trigger writes")

    # scheduler, address map and page policy
    mem_sched_policy = Param.MemSched('fcfs', "Memory scheduling policy")
    addr_mapping = Param.AddrMap('openmap', "Address mapping policy")
    page_policy = Param.PageManage('open', "Page closure management policy")

    # timing behaviour and constraints - all in nanoseconds

    # the amount of time in nanoseconds from issuing an activate command
    # to the data being available in the row buffer for a read/write
    tRCD = Param.Latency("14ns", "RAS to CAS delay")

    # the time from issuing a read/write command to seeing the actual data
    tCL = Param.Latency("14ns", "CAS latency")

    # minimum time between a precharge and subsequent activate
    tRP = Param.Latency("14ns", "Row precharge time")

    # time to complete a burst transfer, typically the burst length
    # divided by two due to the DDR bus, but by making it a parameter
    # it is easier to also evaluate SDR memories like WideIO.
    # This parameter has to account for bus width and burst length.
    # Adjustment also necessary if cache line size is greater than
    # data size read/written by one full burst.
    tBURST = Param.Latency("4ns",
                           "Burst duration (for DDR burst length / 2 cycles)")

    # time taken to complete one refresh cycle (N rows in all banks)
    tRFC = Param.Latency("300ns", "Refresh cycle time")

    # refresh command interval, how often a "ref" command needs
    # to be sent. It is 7.8 us for a 64ms refresh requirement
    tREFI = Param.Latency("7.8us", "Refresh command interval")

    # write-to-read turn around penalty, assumed same as read-to-write
    tWTR = Param.Latency("1ns", "Write to read switching time")

    # Currently unimplemented, unused, deduced or rolled into other params
    ######################################################################

    # the minimum amount of time between a row being activated, and
    # precharged (de-activated)
    # tRAS - assumed to be 3 * tRP

    # tRC  - assumed to be 4 * tRP

    # burst length for an access derived from peerBlockSize

    # @todo: Implement tFAW in the model
    # minimum time window in which a maximum of four activates are
    # allowed to take place
    # tFAW = Param.Latency("30ns", "Four activation window")


