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
"""Interfaces for DDR4 memories

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the `dram` timing
interface.
"""
from m5.objects import DRAMInterface


class DDR4_2400_16x4(DRAMInterface):
    """
    A single DDR4-2400 x64 channel (one command and address bus), with
    timings based on a DDR4-2400 8 Gbit datasheet (Micron MT40A2G4)
    in an 16x4 configuration.
    Total channel capacity is 32GiB
    16 devices/rank * 2 ranks/channel * 1GiB/device = 32GiB/channel
    """

    # size of device
    device_size = "1GiB"

    # 16x4 configuration, 16 devices each with a 4-bit interface
    device_bus_width = 4

    # DDR4 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 512 byte (1K columns x4)
    device_rowbuffer_size = "512B"

    # 16x4 configuration, so 16 devices
    devices_per_rank = 16

    # Match our DDR3 configurations which is dual rank
    ranks_per_channel = 2

    # DDR4 has 2 (x16) or 4 (x4 and x8) bank groups
    # Set to 4 for x4 case
    bank_groups_per_rank = 4

    # DDR4 has 16 banks(x4,x8) and 8 banks(x16) (4 bank groups in all
    # configurations). Currently we do not capture the additional
    # constraints incurred by the bank groups
    banks_per_rank = 16

    # override the default buffer sizes and go for something larger to
    # accommodate the larger bank count
    write_buffer_size = 128
    read_buffer_size = 64

    # 1200 MHz
    tCK = "0.833ns"

    # 8 beats across an x64 interface translates to 4 clocks @ 1200 MHz
    # tBURST is equivalent to the CAS-to-CAS delay (tCCD)
    # With bank group architectures, tBURST represents the CAS-to-CAS
    # delay for bursts to different bank groups (tCCD_S)
    tBURST = "3.332ns"

    # @2400 data rate, tCCD_L is 6 CK
    # CAS-to-CAS delay for bursts to the same bank group
    # tBURST is equivalent to tCCD_S; no explicit parameter required
    # for CAS-to-CAS delay for bursts to different bank groups
    tCCD_L = "5ns"

    # DDR4-2400 17-17-17
    tRCD = "14.16ns"
    tCL = "14.16ns"
    tRP = "14.16ns"
    tRAS = "32ns"

    # RRD_S (different bank group) for 512B page is MAX(4 CK, 3.3ns)
    tRRD = "3.332ns"

    # RRD_L (same bank group) for 512B page is MAX(4 CK, 4.9ns)
    tRRD_L = "4.9ns"

    # tFAW for 512B page is MAX(16 CK, 13ns)
    tXAW = "13.328ns"
    activation_limit = 4
    # tRFC is 350ns
    tRFC = "350ns"

    tWR = "15ns"

    # Here using the average of WTR_S and WTR_L
    tWTR = "5ns"

    # Greater of 4 CK or 7.5 ns
    tRTP = "7.5ns"

    # Default same rank rd-to-wr bus turnaround to 2 CK, @1200 MHz = 1.666 ns
    tRTW = "1.666ns"

    # Default different rank bus delay to 2 CK, @1200 MHz = 1.666 ns
    tCS = "1.666ns"

    # <=85C, half for >85C
    tREFI = "7.8us"

    # active powerdown and precharge powerdown exit time
    tXP = "6ns"

    # self refresh exit time
    # exit delay to ACT, PRE, PREALL, REF, SREF Enter, and PD Enter is:
    # tRFC + 10ns = 340ns
    tXS = "340ns"

    # Current values from datasheet
    IDD0 = "43mA"
    IDD02 = "3mA"
    IDD2N = "34mA"
    IDD3N = "38mA"
    IDD3N2 = "3mA"
    IDD4W = "103mA"
    IDD4R = "110mA"
    IDD5 = "250mA"
    IDD3P1 = "32mA"
    IDD2P1 = "25mA"
    IDD6 = "30mA"
    VDD = "1.2V"
    VDD2 = "2.5V"


class DDR4_2400_8x8(DDR4_2400_16x4):
    """
    A single DDR4-2400 x64 channel (one command and address bus), with
    timings based on a DDR4-2400 8 Gbit datasheet (Micron MT40A1G8)
    in an 8x8 configuration.
    Total channel capacity is 16GiB
    8 devices/rank * 2 ranks/channel * 1GiB/device = 16GiB/channel
    """

    # 8x8 configuration, 8 devices each with an 8-bit interface
    device_bus_width = 8

    # Each device has a page (row buffer) size of 1 Kbyte (1K columns x8)
    device_rowbuffer_size = "1KiB"

    # 8x8 configuration, so 8 devices
    devices_per_rank = 8

    # RRD_L (same bank group) for 1K page is MAX(4 CK, 4.9ns)
    tRRD_L = "4.9ns"

    tXAW = "21ns"

    # Current values from datasheet
    IDD0 = "48mA"
    IDD3N = "43mA"
    IDD4W = "123mA"
    IDD4R = "135mA"
    IDD3P1 = "37mA"


class DDR4_2400_4x16(DDR4_2400_16x4):
    """
    A single DDR4-2400 x64 channel (one command and address bus), with
    timings based on a DDR4-2400 8 Gbit datasheet (Micron MT40A512M16)
    in an 4x16 configuration.
    Total channel capacity is 4GiB
    4 devices/rank * 1 ranks/channel * 1GiB/device = 4GiB/channel
    """

    # 4x16 configuration, 4 devices each with an 16-bit interface
    device_bus_width = 16

    # Each device has a page (row buffer) size of 2 Kbyte (1K columns x16)
    device_rowbuffer_size = "2KiB"

    # 4x16 configuration, so 4 devices
    devices_per_rank = 4

    # Single rank for x16
    ranks_per_channel = 1

    # DDR4 has 2 (x16) or 4 (x4 and x8) bank groups
    # Set to 2 for x16 case
    bank_groups_per_rank = 2

    # DDR4 has 16 banks(x4,x8) and 8 banks(x16) (4 bank groups in all
    # configurations). Currently we do not capture the additional
    # constraints incurred by the bank groups
    banks_per_rank = 8

    # RRD_S (different bank group) for 2K page is MAX(4 CK, 5.3ns)
    tRRD = "5.3ns"

    # RRD_L (same bank group) for 2K page is MAX(4 CK, 6.4ns)
    tRRD_L = "6.4ns"

    tXAW = "30ns"

    # Current values from datasheet
    IDD0 = "80mA"
    IDD02 = "4mA"
    IDD2N = "34mA"
    IDD3N = "47mA"
    IDD4W = "228mA"
    IDD4R = "243mA"
    IDD5 = "280mA"
    IDD3P1 = "41mA"
