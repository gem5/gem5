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

"""Interfaces for LPDDR5 memory devices

These memory "interfaces" contain the timing,energy,etc parameters for each
memory type and are usually based on datasheets for the memory devices.

You can use these interfaces in the MemCtrl object as the ``dram`` timing
interface.
"""

from m5.objects import DRAMInterface


class LPDDR5_5500_1x16_BG_BL32(DRAMInterface):
    """
    A single LPDDR5 x16 interface (one command/address bus)
    for a single x16 channel with default timings based on
    initial JEDEC specification.

    Starting with 5.5Gbps data rates and 8Gbit die.

    Configuring for 16-bank mode with bank-group architecture
    burst of 32, which means bursts can be interleaved.
    """

    # Increase buffer size to account for more bank resources
    read_buffer_size = 64

    # Set page policy to better suit DMC Huxley
    page_policy = "close_adaptive"

    # 16-bit channel interface
    device_bus_width = 16

    # LPDDR5 is a BL16 or BL32 device
    # With BG mode, BL16 and BL32 are supported
    # Use BL32 for higher command bandwidth
    burst_length = 32

    # size of device in bytes
    device_size = "1GiB"

    # 2KiB page with BG mode
    device_rowbuffer_size = "2KiB"

    # Use a 1x16 configuration
    devices_per_rank = 1

    # Use a single rank
    ranks_per_channel = 1

    # LPDDR5 supports configurable bank options
    # 8B  : BL32, all frequencies
    # 16B : BL32 or BL16, <=3.2Gbps
    # 16B with Bank Group Arch (4B/BG): BL32 or BL16, >3.2Gbps
    # Initial configuration will have 16 banks with Bank Group Arch
    # to maximim resources and enable higher data rates
    banks_per_rank = 16
    bank_groups_per_rank = 4

    # 5.5Gb/s DDR with 4:1 WCK:CK ratio for 687.5 MHz CK
    tCK = "1.455ns"

    # Greater of 2 CK or 18ns
    tRCD = "18ns"

    # Base RL is 16 CK @ 687.5 MHz = 23.28ns
    tCL = "23.280ns"

    # Greater of 2 CK or 18ns
    tRP = "18ns"

    # Greater of 3 CK or 42ns
    tRAS = "42ns"

    # Greater of 3 CK or 34ns
    tWR = "34ns"

    # active powerdown and precharge powerdown exit time
    # Greater of 3 CK or 7ns
    tXP = "7ns"

    # self refresh exit time (tRFCab + 7.5ns)
    tXS = "217.5ns"

    # Greater of 2 CK or 7.5 ns minus 2 CK
    tRTP = "4.59ns"

    # With BG architecture, burst of 32 transferred in two 16-beat
    # sub-bursts, with a 16-beat gap in between.
    # Each 16-beat sub-burst is 8 WCK @2.75 GHz or 2 CK @ 687.5 MHz
    # tBURST is the delay to transfer the Bstof32 =  6 CK @ 687.5 MHz
    tBURST = "8.73ns"
    # can interleave a Bstof32 from another bank group at tBURST_MIN
    # 16-beats is 8 WCK @2.75 GHz or 2 CK @ 687.5 MHz
    tBURST_MIN = "2.91ns"
    # tBURST_MAX is the maximum burst delay for same bank group timing
    # this is 8 CK @ 687.5 MHz
    tBURST_MAX = "11.64ns"

    # 8 CK @ 687.5 MHz
    tCCD_L = "11.64ns"

    # LPDDR5, 8 Gbit/channel for 280ns tRFCab
    tRFC = "210ns"
    tREFI = "3.9us"

    # Greater of 4 CK or 6.25 ns
    tWTR = "6.25ns"
    # Greater of 4 CK or 12 ns
    tWTR_L = "12ns"

    # Required RD-to-WR timing is RL+ BL/n + tWCKDQ0/tCK - WL
    # tWCKDQ0/tCK will be 1 CK for most cases
    # For gem5 RL = WL and BL/n is already accounted for with tBURST
    # Result is and additional 1 CK is required
    tRTW = "1.455ns"

    # Default different rank bus delay to 2 CK, @687.5 MHz = 2.91 ns
    tCS = "2.91ns"

    # 2 CK
    tPPD = "2.91ns"

    # Greater of 2 CK or 5 ns
    tRRD = "5ns"
    tRRD_L = "5ns"

    # With Bank Group Arch mode tFAW is 20 ns
    tXAW = "20ns"
    activation_limit = 4

    # at 5Gbps, 4:1 WCK to CK ratio required
    # 2 data beats per WCK (DDR) -> 8 per CK
    beats_per_clock = 8

    # 2 cycles required to send activate command
    # 2 command phases can be sent back-to-back or
    # with a gap up to tAAD = 8 CK
    two_cycle_activate = True
    tAAD = "11.640ns"

    data_clock_sync = True


class LPDDR5_5500_1x16_BG_BL16(LPDDR5_5500_1x16_BG_BL32):
    """
    A single LPDDR5 x16 interface (one command/address bus)
    for a single x16 channel with default timings based on
    initial JEDEC specification
    Starting with 5.5Gbps data rates and 8Gbit die
    Configuring for 16-bank mode with bank-group architecture, burst of 16
    """

    # LPDDR5 is a BL16 or BL32 device
    # With BG mode, BL16 and BL32 are supported
    # Use BL16 for smaller access granularity
    burst_length = 16

    # For Bstof16 with BG arch, 2 CK @ 687.5 MHz with 4:1 clock ratio
    tBURST = "2.91ns"
    tBURST_MIN = "2.91ns"
    # For Bstof16 with BG arch, 4 CK @ 687.5 MHz with 4:1 clock ratio
    tBURST_MAX = "5.82ns"

    # 4 CK @ 687.5 MHz
    tCCD_L = "5.82ns"


class LPDDR5_5500_1x16_8B_BL32(LPDDR5_5500_1x16_BG_BL32):
    """
    A single LPDDR5 x16 interface (one command/address bus)
    for a single x16 channel with default timings based on
    initial JEDEC specification.

    Starting with 5.5Gbps data rates and 8Gbit die.

    Configuring for 8-bank mode, burst of 32.
    """

    # 4KiB page with 8B mode
    device_rowbuffer_size = "4KiB"

    # LPDDR5 supports configurable bank options
    # 8B  : BL32, all frequencies
    # 16B : BL32 or BL16, <=3.2Gbps
    # 16B with Bank Group Arch (4B/BG): BL32 or BL16, >3.2Gbps
    # Select 8B
    banks_per_rank = 8
    bank_groups_per_rank = 0

    # For Bstof32 with 8B mode, 4 CK @ 687.5 MHz with 4:1 clock ratio
    tBURST = "5.82ns"
    tBURST_MIN = "5.82ns"
    tBURST_MAX = "5.82ns"

    # Greater of 4 CK or 12 ns
    tWTR = "12ns"

    # Greater of 2 CK or 10 ns
    tRRD = "10ns"

    # With 8B mode tFAW is 40 ns
    tXAW = "40ns"
    activation_limit = 4

    # Reset BG arch timing for 8B mode
    tCCD_L = "0ns"
    tRRD_L = "0ns"
    tWTR_L = "0ns"


class LPDDR5_6400_1x16_BG_BL32(LPDDR5_5500_1x16_BG_BL32):
    """
    A single LPDDR5 x16 interface (one command/address bus)
    for a single x16 channel with default timings based on
    initial JEDEC specification.

    6.4Gbps data rates and 8Gbit die.

    Configuring for 16-bank mode with bank-group architecture
    burst of 32, which means bursts can be interleaved.
    """

    # 5.5Gb/s DDR with 4:1 WCK:CK ratio for 687.5 MHz CK
    tCK = "1.25ns"

    # Base RL is 17 CK @ 800 MHz = 21.25ns
    tCL = "21.25ns"

    # With BG architecture, burst of 32 transferred in two 16-beat
    # sub-bursts, with a 16-beat gap in between.
    # Each 16-beat sub-burst is 8 WCK @3.2 GHz or 2 CK @ 800 MHz
    # tBURST is the delay to transfer the Bstof32 =  6 CK @ 800 MHz
    tBURST = "7.5ns"
    # can interleave a Bstof32 from another bank group at tBURST_MIN
    # 16-beats is 8 WCK @2.3 GHz or 2 CK @ 800 MHz
    tBURST_MIN = "2.5ns"
    # tBURST_MAX is the maximum burst delay for same bank group timing
    # this is 8 CK @ 800 MHz
    tBURST_MAX = "10ns"

    # 8 CK @ 800 MHz
    tCCD_L = "10ns"

    # Required RD-to-WR timing is RL+ BL/n + tWCKDQ0/tCK - WL
    # tWCKDQ0/tCK will be 1 CK for most cases
    # For gem5 RL = WL and BL/n is already accounted for with tBURST
    # Result is and additional 1 CK is required
    tRTW = "1.25ns"

    # Default different rank bus delay to 2 CK, @687.5 MHz = 2.5 ns
    tCS = "2.5ns"

    # 2 CK
    tPPD = "2.5ns"

    # 2 command phases can be sent back-to-back or
    # with a gap up to tAAD = 8 CK
    tAAD = "10ns"


class LPDDR5_6400_1x16_BG_BL16(LPDDR5_6400_1x16_BG_BL32):
    """
    A single LPDDR5 x16 interface (one command/address bus)
    for a single x16 channel with default timings based on initial
    JEDEC specifcation.

    6.4Gbps data rates and 8Gbit die.

    Configuring for 16-bank mode with bank-group architecture, burst of 16.
    """

    # LPDDR5 is a BL16 or BL32 device
    # With BG mode, BL16 and BL32 are supported
    # Use BL16 for smaller access granularity
    burst_length = 16

    # For Bstof16 with BG arch, 2 CK @ 800 MHz with 4:1 clock ratio
    tBURST = "2.5ns"
    tBURST_MIN = "2.5ns"
    # For Bstof16 with BG arch, 4 CK @ 800 MHz with 4:1 clock ratio
    tBURST_MAX = "5ns"

    # 4 CK @ 800 MHz
    tCCD_L = "5ns"


class LPDDR5_6400_1x16_8B_BL32(LPDDR5_6400_1x16_BG_BL32):
    """
    A single LPDDR5 x16 interface (one command/address bus)
    for a single x16 channel with default timings based on
    initial JEDEC specification.

    6.4Gbps data rates and 8Gbit die.

    Configuring for 8-bank mode, burst of 32.
    """

    # 4KiB page with 8B mode
    device_rowbuffer_size = "4KiB"

    # LPDDR5 supports configurable bank options
    # 8B  : BL32, all frequencies
    # 16B : BL32 or BL16, <=3.2Gbps
    # 16B with Bank Group Arch (4B/BG): BL32 or BL16, >3.2Gbps
    # Select 8B
    banks_per_rank = 8
    bank_groups_per_rank = 0

    # For Bstof32 with 8B mode, 4 CK @ 800 MHz with 4:1 clock ratio
    tBURST = "5ns"
    tBURST_MIN = "5ns"
    tBURST_MAX = "5ns"

    # Greater of 4 CK or 12 ns
    tWTR = "12ns"

    # Greater of 2 CK or 10 ns
    tRRD = "10ns"

    # With 8B mode tFAW is 40 ns
    tXAW = "40ns"
    activation_limit = 4

    # Reset BG arch timing for 8B mode
    tCCD_L = "0ns"
    tRRD_L = "0ns"
    tWTR_L = "0ns"
