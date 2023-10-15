# Copyright (c) 2012-2021 Arm Limited
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

"""Interfaces for HBM memory devices

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the `dram` timing
interface.
"""

from m5.objects import DRAMInterface


class HBM_1000_4H_1x128(DRAMInterface):
    """
    A single HBM x128 interface (one command and address bus), with
    default timings based on data publically released
    ("HBM: Memory Solution for High Performance Processors", MemCon, 2014),
    IDD measurement values, and by extrapolating data from other classes.
    Architecture values based on published HBM spec
    A 4H stack is defined, 2Gb per die for a total of 1GiB of memory.

    **IMPORTANT**
    HBM gen1 supports up to 8 128-bit physical channels
    Configuration defines a single channel, with the capacity
    set to (full_ stack_capacity / 8) based on 2Gb dies
    To use all 8 channels, set 'channels' parameter to 8 in
    system configuration
    """

    # 128-bit interface legacy mode
    device_bus_width = 128

    # HBM supports BL4 and BL2 (legacy mode only)
    burst_length = 4

    # size of channel in bytes, 4H stack of 2Gb dies is 1GiB per stack;
    # with 8 channels, 128MiB per channel
    device_size = "128MiB"

    device_rowbuffer_size = "2KiB"

    # 1x128 configuration
    devices_per_rank = 1

    # HBM does not have a CS pin; set rank to 1
    ranks_per_channel = 1

    # HBM has 8 or 16 banks depending on capacity
    # 2Gb dies have 8 banks
    banks_per_rank = 8

    # depending on frequency, bank groups may be required
    # will always have 4 bank groups when enabled
    # current specifications do not define the minimum frequency for
    # bank group architecture
    # setting bank_groups_per_rank to 0 to disable until range is defined
    bank_groups_per_rank = 0

    # 500 MHz for 1Gbps DDR data rate
    tCK = "2ns"

    # use values from IDD measurement in JEDEC spec
    # use tRP value for tRCD and tCL similar to other classes
    tRP = "15ns"
    tRCD = "15ns"
    tCL = "15ns"
    tRAS = "33ns"

    # BL2 and BL4 supported, default to BL4
    # DDR @ 500 MHz means 4 * 2ns / 2 = 4ns
    tBURST = "4ns"

    # value for 2Gb device from JEDEC spec
    tRFC = "160ns"

    # value for 2Gb device from JEDEC spec
    tREFI = "3.9us"

    # extrapolate the following from LPDDR configs, using ns values
    # to minimize burst length, prefetch differences
    tWR = "18ns"
    tRTP = "7.5ns"
    tWTR = "10ns"

    # start with 2 cycles turnaround, similar to other memory classes
    # could be more with variations across the stack
    tRTW = "4ns"

    # single rank device, set to 0
    tCS = "0ns"

    # from MemCon example, tRRD is 4ns with 2ns tCK
    tRRD = "4ns"

    # from MemCon example, tFAW is 30ns with 2ns tCK
    tXAW = "30ns"
    activation_limit = 4

    # 4tCK
    tXP = "8ns"

    # start with tRFC + tXP -> 160ns + 8ns = 168ns
    tXS = "168ns"


class HBM_1000_4H_1x64(HBM_1000_4H_1x128):
    """
    A single HBM x64 interface (one command and address bus), with
    default timings based on HBM gen1 and data publically released
    A 4H stack is defined, 8Gb per die for a total of 4GiB of memory.
    Note: This defines a pseudo-channel with a unique controller
    instantiated per pseudo-channel
    Stay at same IO rate (1Gbps) to maintain timing relationship with
    HBM gen1 class (HBM_1000_4H_x128) where possible

    **IMPORTANT**
    For HBM gen2 with pseudo-channel mode, configure 2X channels.
    Configuration defines a single pseudo channel, with the capacity
    set to (full_ stack_capacity / 16) based on 8Gb dies
    To use all 16 pseudo channels, set 'channels' parameter to 16 in
    system configuration
    """

    # 64-bit pseudo-channel interface
    device_bus_width = 64

    # HBM pseudo-channel only supports BL4
    burst_length = 4

    # size of channel in bytes, 4H stack of 8Gb dies is 4GiB per stack;
    # with 16 channels, 256MiB per channel
    device_size = "256MiB"

    # page size is halved with pseudo-channel; maintaining the same same number
    # of rows per pseudo-channel with 2X banks across 2 channels
    device_rowbuffer_size = "1KiB"

    # HBM has 8 or 16 banks depending on capacity
    # Starting with 4Gb dies, 16 banks are defined
    banks_per_rank = 16

    # reset tRFC for larger, 8Gb device
    # use HBM1 4Gb value as a starting point
    tRFC = "260ns"

    # start with tRFC + tXP -> 160ns + 8ns = 168ns
    tXS = "268ns"
    # Default different rank bus delay to 2 CK, @1000 MHz = 2 ns
    tCS = "2ns"
    tREFI = "3.9us"

    # active powerdown and precharge powerdown exit time
    tXP = "10ns"

    # self refresh exit time
    tXS = "65ns"


# A single HBM2 x64 interface (tested with HBMCtrl in gem5)
# to be used as a single pseudo channel. The timings are based
# on HBM gen2 specifications. 4H stack, 8Gb per die and total capacity
# of 4GiB.
class HBM_2000_4H_1x64(DRAMInterface):
    # 64-bit interface for a single pseudo channel
    device_bus_width = 64

    # HBM2 supports BL4
    burst_length = 4

    # size of channel in bytes, 4H stack of 8Gb dies is 4GiB per stack;
    # with 16 pseudo channels, 256MiB per pseudo channel
    device_size = "256MiB"

    device_rowbuffer_size = "1KiB"

    # 1x128 configuration
    devices_per_rank = 1

    ranks_per_channel = 1

    banks_per_rank = 16
    bank_groups_per_rank = 4

    # 1000 MHz for 2Gbps DDR data rate
    tCK = "1ns"

    tRP = "14ns"

    tCCD_L = "3ns"

    tRCD = "12ns"
    tRCD_WR = "6ns"
    tCL = "18ns"
    tCWL = "7ns"
    tRAS = "28ns"

    # BL4 in pseudo channel mode
    # DDR @ 1000 MHz means 4 * 1ns / 2 = 2ns
    tBURST = "2ns"

    # value for 2Gb device from JEDEC spec
    tRFC = "220ns"

    # value for 2Gb device from JEDEC spec
    tREFI = "3.9us"

    tWR = "14ns"
    tRTP = "5ns"
    tWTR = "4ns"
    tWTR_L = "9ns"
    tRTW = "18ns"

    # tAAD from RBus
    tAAD = "1ns"

    # single rank device, set to 0
    tCS = "0ns"

    tRRD = "4ns"
    tRRD_L = "6ns"

    # for a single pseudo channel
    tXAW = "16ns"
    activation_limit = 4

    # 4tCK
    tXP = "8ns"

    # start with tRFC + tXP -> 160ns + 8ns = 168ns
    tXS = "216ns"

    page_policy = "close_adaptive"

    read_buffer_size = 64
    write_buffer_size = 64

    two_cycle_activate = True
