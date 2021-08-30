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

"""Interfaces for DDR3 memories

These memory "interfaces" contain the timing, energy, etc. parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the `dram` timing
interface.
"""

from m5.objects import DRAMInterface


class DDR3_1600_8x8(DRAMInterface):
    """
    A single DDR3-1600 x64 channel (one command and address bus), with
    timings based on a DDR3-1600 4 Gbit datasheet (Micron MT41J512M8) in
    an 8x8 configuration.

    """

    # size of device in bytes
    device_size = "512MiB"

    # 8x8 configuration, 8 devices each with an 8-bit interface
    device_bus_width = 8

    # DDR3 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 1 Kbyte (1K columns x8)
    device_rowbuffer_size = "1KiB"

    # 8x8 configuration, so 8 devices
    devices_per_rank = 8

    # Use two ranks
    ranks_per_channel = 2

    # DDR3 has 8 banks in all configurations
    banks_per_rank = 8

    # 800 MHz
    tCK = "1.25ns"

    # 8 beats across an x64 interface translates to 4 clocks @ 800 MHz
    tBURST = "5ns"

    # DDR3-1600 11-11-11
    tRCD = "13.75ns"
    tCL = "13.75ns"
    tRP = "13.75ns"
    tRAS = "35ns"
    tRRD = "6ns"
    tXAW = "30ns"
    activation_limit = 4
    tRFC = "260ns"

    tWR = "15ns"

    # Greater of 4 CK or 7.5 ns
    tWTR = "7.5ns"

    # Greater of 4 CK or 7.5 ns
    tRTP = "7.5ns"

    # Default same rank rd-to-wr bus turnaround to 2 CK, @800 MHz = 2.5 ns
    tRTW = "2.5ns"

    # Default different rank bus delay to 2 CK, @800 MHz = 2.5 ns
    tCS = "2.5ns"

    # <=85C, half for >85C
    tREFI = "7.8us"

    # active powerdown and precharge powerdown exit time
    tXP = "6ns"

    # self refresh exit time
    tXS = "270ns"

    # Current values from datasheet Die Rev E,J
    IDD0 = "55mA"
    IDD2N = "32mA"
    IDD3N = "38mA"
    IDD4W = "125mA"
    IDD4R = "157mA"
    IDD5 = "235mA"
    IDD3P1 = "38mA"
    IDD2P1 = "32mA"
    IDD6 = "20mA"
    VDD = "1.5V"


class DDR3_2133_8x8(DDR3_1600_8x8):
    """
    A single DDR3-2133 x64 channel refining a selected subset of the
    options for the DDR-1600 configuration, based on the same DDR3-1600
    4 Gbit datasheet (Micron MT41J512M8). Most parameters are kept
    consistent across the two configurations.
    """

    # 1066 MHz
    tCK = "0.938ns"

    # 8 beats across an x64 interface translates to 4 clocks @ 1066 MHz
    tBURST = "3.752ns"

    # DDR3-2133 14-14-14
    tRCD = "13.09ns"
    tCL = "13.09ns"
    tRP = "13.09ns"
    tRAS = "33ns"
    tRRD = "5ns"
    tXAW = "25ns"

    # Current values from datasheet
    IDD0 = "70mA"
    IDD2N = "37mA"
    IDD3N = "44mA"
    IDD4W = "157mA"
    IDD4R = "191mA"
    IDD5 = "250mA"
    IDD3P1 = "44mA"
    IDD2P1 = "43mA"
    IDD6 = "20mA"
    VDD = "1.5V"
