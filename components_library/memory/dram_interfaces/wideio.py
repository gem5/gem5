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

"""Interfaces for WideIO memory devices

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the `dram` timing
interface.
"""

from m5.objects import DRAMInterface


class WideIO_200_1x128(DRAMInterface):
    """
    A single WideIO x128 interface (one command and address bus), with
    default timings based on an estimated WIO-200 8 Gbit part.
    """

    # No DLL for WideIO
    dll = False

    # size of device
    device_size = "1024MiB"

    # 1x128 configuration, 1 device with a 128-bit interface
    device_bus_width = 128

    # This is a BL4 device
    burst_length = 4

    # Each device has a page (row buffer) size of 4KB
    # (this depends on the memory density)
    device_rowbuffer_size = "4KiB"

    # 1x128 configuration, so 1 device
    devices_per_rank = 1

    # Use one rank for a one-high die stack
    ranks_per_channel = 1

    # WideIO has 4 banks in all configurations
    banks_per_rank = 4

    # 200 MHz
    tCK = "5ns"

    # WIO-200
    tRCD = "18ns"
    tCL = "18ns"
    tRP = "18ns"
    tRAS = "42ns"
    tWR = "15ns"
    # Read to precharge is same as the burst
    tRTP = "20ns"

    # 4 beats across an x128 SDR interface translates to 4 clocks @ 200 MHz.
    # Note this is a BL4 SDR device.
    tBURST = "20ns"

    # WIO 8 Gb
    tRFC = "210ns"

    # WIO 8 Gb, <=85C, half for >85C
    tREFI = "3.9us"

    # Greater of 2 CK or 15 ns, 2 CK @ 200 MHz = 10 ns
    tWTR = "15ns"

    # Default same rank rd-to-wr bus turnaround to 2 CK, @200 MHz = 10 ns
    tRTW = "10ns"

    # Default different rank bus delay to 2 CK, @200 MHz = 10 ns
    tCS = "10ns"

    # Activate to activate irrespective of density and speed grade
    tRRD = "10.0ns"

    # Two instead of four activation window
    tXAW = "50ns"
    activation_limit = 2

    # The WideIO specification does not provide current information
