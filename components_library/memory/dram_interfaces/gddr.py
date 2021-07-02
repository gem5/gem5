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

"""Interfaces for GDDR memory devices

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the `dram` timing
interface.
"""

from m5.objects import DRAMInterface


class GDDR5_4000_2x32(DRAMInterface):
    """
    A single GDDR5 x64 interface, with default timings based on a GDDR5-4000
    1 Gbit part (SK Hynix H5GQ1H24AFR) in a 2x32 configuration.
    """

    # size of device
    device_size = "128MiB"

    # 2x32 configuration, 1 device with a 32-bit interface
    device_bus_width = 32

    # GDDR5 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 2Kbits (256Bytes)
    device_rowbuffer_size = "256B"

    # 2x32 configuration, so 2 devices
    devices_per_rank = 2

    # assume single rank
    ranks_per_channel = 1

    # GDDR5 has 4 bank groups
    bank_groups_per_rank = 4

    # GDDR5 has 16 banks with 4 bank groups
    banks_per_rank = 16

    # 1000 MHz
    tCK = "1ns"

    # 8 beats across an x64 interface translates to 2 clocks @ 1000 MHz
    # Data bus runs @2000 Mhz => DDR ( data runs at 4000 MHz )
    # 8 beats at 4000 MHz = 2 beats at 1000 MHz
    # tBURST is equivalent to the CAS-to-CAS delay (tCCD)
    # With bank group architectures, tBURST represents the CAS-to-CAS
    # delay for bursts to different bank groups (tCCD_S)
    tBURST = "2ns"

    # @1000MHz data rate, tCCD_L is 3 CK
    # CAS-to-CAS delay for bursts to the same bank group
    # tBURST is equivalent to tCCD_S; no explicit parameter required
    # for CAS-to-CAS delay for bursts to different bank groups
    tCCD_L = "3ns"

    tRCD = "12ns"

    # tCL is not directly found in datasheet and assumed equal tRCD
    tCL = "12ns"

    tRP = "12ns"
    tRAS = "28ns"

    # RRD_S (different bank group)
    # RRD_S is 5.5 ns in datasheet.
    # rounded to the next multiple of tCK
    tRRD = "6ns"

    # RRD_L (same bank group)
    # RRD_L is 5.5 ns in datasheet.
    # rounded to the next multiple of tCK
    tRRD_L = "6ns"

    tXAW = "23ns"

    # tXAW < 4 x tRRD.
    # Therefore, activation limit is set to 0
    activation_limit = 0

    tRFC = "65ns"
    tWR = "12ns"

    # Here using the average of WTR_S and WTR_L
    tWTR = "5ns"

    # Read-to-Precharge 2 CK
    tRTP = "2ns"

    # Assume 2 cycles
    tRTW = "2ns"
