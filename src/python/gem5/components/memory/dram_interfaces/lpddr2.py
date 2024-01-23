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

"""Interfaces for LPDDR2 memory devices

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the ``dram`` timing
interface.
"""

from m5.objects import DRAMInterface


class LPDDR2_S4_1066_1x32(DRAMInterface):
    """
    A single LPDDR2-S4 x32 interface (one command/address bus), with
    default timings based on a LPDDR2-1066 4 Gbit part (Micron MT42L128M32D1)
    in a 1x32 configuration.
    """

    # No DLL in LPDDR2
    dll = False

    # size of device
    device_size = "512MiB"

    # 1x32 configuration, 1 device with a 32-bit interface
    device_bus_width = 32

    # LPDDR2_S4 is a BL4 and BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 1KB
    # (this depends on the memory density)
    device_rowbuffer_size = "1KiB"

    # 1x32 configuration, so 1 device
    devices_per_rank = 1

    # Use a single rank
    ranks_per_channel = 1

    # LPDDR2-S4 has 8 banks in all configurations
    banks_per_rank = 8

    # 533 MHz
    tCK = "1.876ns"

    # Fixed at 15 ns
    tRCD = "15ns"

    # 8 CK read latency, 4 CK write latency @ 533 MHz, 1.876 ns cycle time
    tCL = "15ns"

    # Pre-charge one bank 15 ns (all banks 18 ns)
    tRP = "15ns"

    tRAS = "42ns"
    tWR = "15ns"

    tRTP = "7.5ns"

    # 8 beats across an x32 DDR interface translates to 4 clocks @ 533 MHz.
    # Note this is a BL8 DDR device.
    # Requests larger than 32 bytes are broken down into multiple requests
    # in the controller
    tBURST = "7.5ns"

    # LPDDR2-S4, 4 Gbit
    tRFC = "130ns"
    tREFI = "3.9us"

    # active powerdown and precharge powerdown exit time
    tXP = "7.5ns"

    # self refresh exit time
    tXS = "140ns"

    # Irrespective of speed grade, tWTR is 7.5 ns
    tWTR = "7.5ns"

    # Default same rank rd-to-wr bus turnaround to 2 CK, @533 MHz = 3.75 ns
    tRTW = "3.75ns"

    # Default different rank bus delay to 2 CK, @533 MHz = 3.75 ns
    tCS = "3.75ns"

    # Activate to activate irrespective of density and speed grade
    tRRD = "10.0ns"

    # Irrespective of density, tFAW is 50 ns
    tXAW = "50ns"
    activation_limit = 4

    # Current values from datasheet
    IDD0 = "15mA"
    IDD02 = "70mA"
    IDD2N = "2mA"
    IDD2N2 = "30mA"
    IDD3N = "2.5mA"
    IDD3N2 = "30mA"
    IDD4W = "10mA"
    IDD4W2 = "190mA"
    IDD4R = "3mA"
    IDD4R2 = "220mA"
    IDD5 = "40mA"
    IDD52 = "150mA"
    IDD3P1 = "1.2mA"
    IDD3P12 = "8mA"
    IDD2P1 = "0.6mA"
    IDD2P12 = "0.8mA"
    IDD6 = "1mA"
    IDD62 = "3.2mA"
    VDD = "1.8V"
    VDD2 = "1.2V"
