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

"""Interfaces for LPDDR3 memory devices

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the ``dram`` timing
interface.
"""

from m5.objects import DRAMInterface


class LPDDR3_1600_1x32(DRAMInterface):
    """
    A single LPDDR3 x32 interface (one command/address bus), with default
    timings based on a LPDDR3-1600 4 Gbit part (Micron EDF8132A1MC) in a 1x32
    configuration.
    """

    # No DLL for LPDDR3
    dll = False

    # size of device
    device_size = "512MiB"

    # 1x32 configuration, 1 device with a 32-bit interface
    device_bus_width = 32

    # LPDDR3 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 4KiB
    device_rowbuffer_size = "4KiB"

    # 1x32 configuration, so 1 device
    devices_per_rank = 1

    # Technically the datasheet is a dual-rank package, but for
    # comparison with the LPDDR2 config we stick to a single rank
    ranks_per_channel = 1

    # LPDDR3 has 8 banks in all configurations
    banks_per_rank = 8

    # 800 MHz
    tCK = "1.25ns"

    tRCD = "18ns"

    # 12 CK read latency, 6 CK write latency @ 800 MHz, 1.25 ns cycle time
    tCL = "15ns"

    tRAS = "42ns"
    tWR = "15ns"

    # Greater of 4 CK or 7.5 ns, 4 CK @ 800 MHz = 5 ns
    tRTP = "7.5ns"

    # Pre-charge one bank 18 ns (all banks 21 ns)
    tRP = "18ns"

    # 8 beats across a x32 DDR interface translates to 4 clocks @ 800 MHz.
    # Note this is a BL8 DDR device.
    # Requests larger than 32 bytes are broken down into multiple requests
    # in the controller
    tBURST = "5ns"

    # LPDDR3, 4 Gb
    tRFC = "130ns"
    tREFI = "3.9us"

    # active powerdown and precharge powerdown exit time
    tXP = "7.5ns"

    # self refresh exit time
    tXS = "140ns"

    # Irrespective of speed grade, tWTR is 7.5 ns
    tWTR = "7.5ns"

    # Default same rank rd-to-wr bus turnaround to 2 CK, @800 MHz = 2.5 ns
    tRTW = "2.5ns"

    # Default different rank bus delay to 2 CK, @800 MHz = 2.5 ns
    tCS = "2.5ns"

    # Activate to activate irrespective of density and speed grade
    tRRD = "10.0ns"

    # Irrespective of size, tFAW is 50 ns
    tXAW = "50ns"
    activation_limit = 4

    # Current values from datasheet
    IDD0 = "8mA"
    IDD02 = "60mA"
    IDD2N = "0.8mA"
    IDD2N2 = "26mA"
    IDD3N = "2mA"
    IDD3N2 = "34mA"
    IDD4W = "2mA"
    IDD4W2 = "190mA"
    IDD4R = "2mA"
    IDD4R2 = "230mA"
    IDD5 = "28mA"
    IDD52 = "150mA"
    IDD3P1 = "1.4mA"
    IDD3P12 = "11mA"
    IDD2P1 = "0.8mA"
    IDD2P12 = "1.8mA"
    IDD6 = "0.5mA"
    IDD62 = "1.8mA"
    VDD = "1.8V"
    VDD2 = "1.2V"
