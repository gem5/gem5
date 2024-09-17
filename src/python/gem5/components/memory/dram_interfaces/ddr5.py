# Copyright (c) 2023 The Regents of the University of California
# All Rights Reserved.
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

"""Interfaces for DDR5 memories
"""

from m5.objects import DRAMInterface


# A single DDR5-4400 32bit channel (4x8 configuration)
# A DDR5 DIMM is made up of two (32 bit) channels.
# Following configuration is modeling only a single 32bit channel.
# Timings are based on Micron data sheet:
# https://media-www.micron.com/-/media/client/global/
# documents/products/data-sheet/dram/ddr5/ddr5_sdram_core.pdf
# Configuration: 4Gbx8 devices (32Gb addressing)
# Maximum bandwidth of DDR5_4400_4x8 (4400 MT/s) can be 17.6GB/s
class DDR5_4400_4x8(DRAMInterface):
    # size of a single device
    device_size = "512MiB"

    # single channel of 32bit width
    # requires 8-bit wide 4 devices
    device_bus_width = 8

    # DDR5 is a BL16 device
    burst_length = 16

    # Each device has a page (row buffer) size of 256B
    # Four devices lead to a page size of 1KiB
    device_rowbuffer_size = "256B"

    # 4Gbx8 configuration
    devices_per_rank = 4

    ranks_per_channel = 1

    # 4Gbx8 configuration of DDR5 has 8 bank groups,
    # 4 banks per bank group and 32 banks in total
    bank_groups_per_rank = 8
    banks_per_rank = 32

    write_buffer_size = 64
    read_buffer_size = 64

    # For 4400 MT/s
    tCK = "0.454ns"

    # 16 beats across an x32 interface translates to 8 clocks @ 2200 MHz
    tBURST = "3.632ns"

    # For 4400 MT/s, the number is max(8nCK, 5ns)
    tCCD_L = "5ns"

    # page 389 of the data sheet
    tRCD = "14.545ns"
    tCL = "14.545ns"
    # tCWL = tCL - 2(nCK)
    tCWL = "13.637ns"
    tRP = "14.545ns"
    tRAS = "32ns"

    # RRD_S (different bank group) : 8nCK
    tRRD = "3.632ns"

    # RRD_L (same bank group) is MAX(8nCK, 5ns) for 1KiB page
    tRRD_L = "5ns"

    # tFAW for 1KiB page is MAX(32nCK, 14.545ns)
    tXAW = "14.545ns"
    activation_limit = 4

    # Note: could not find the rank to rank delay
    # from the datasheet, but, since we are modeling
    # a single rank device, it should not matter.
    # Using the DDR4 number as default (i.e. ~2tCK)
    tCS = "1ns"

    # tRFC (Normal) for 16Gb device is 295ns
    tRFC = "295ns"

    tPPD = "0.908ns"  # 2nCK
    tWR = "30ns"

    # Rd/Wr turnaround timings
    ###################################################################
    # Note: gem5 adds tBURST separately while calculting rd--rd/wr or
    # wr--wr/rd delays so we can ignore tBURST factor from the following
    # equations while calculating the final value of the timing parameter
    ####################################################################
    # From the datasheet
    # (1) tCCD_L_RTW =
    # CL - CWL + RBL/2 + 2 tCK - (Read DQS offset) + (tRPST - 0.5 tCK) + tWPRE
    # where CWL = CL-2, RBL/2 = tBURST, Read DQS offset = 1ck, tRPST = 1.5tCK
    # Therefore, tCCD_L_RTW =
    # (14.545 - 13.637) + (2*0.454) - 0.454 +
    # ((1.5*0.454)-(0.5*0.454) + (1.5*0.454) = 2.497ns

    # (2) tCCD_S_RTW =
    # CL - CWL + RBL/2 + 2 tCK - (Read DQS offset) + (tRPST - 0.5 tCK) + tWPRE
    # Therefore, tCCD_S_RTW = tCCD_L_RTW = 2.497ns

    # (3) tCCD_L_WTR =
    # CWL + WBL/2 + max(16nCK,10ns)
    # where WBL/2 = tBURST
    # Therefore,
    # tCCD_L_WTR = 13.637+10 = 23.637ns

    # (4) tCCD_S_WTR =
    # CWL + WBL/2 + max(4nCK,2.5ns)
    # where WBL/2 = tBURST
    # Therefore,
    # tCCD_S_WTR = 13.637+2.5 = 16.137ns

    tRTW = "2.497ns"
    tWTR_L = "23.637ns"
    tWTR = "16.137ns"

    # tRTP : max(12nCK, 7.5ns)
    tRTP = "7.5ns"

    # <=85C, half for >85C
    tREFI = "3.9us"

    # active powerdown and precharge powerdown exit time max(7.5ns, 8nCK)
    tXP = "7.5ns"

    # self refresh exit time
    # According to the datasheet tXS = tRFC = 295ns (normal Refresh mode)
    tXS = "295ns"

    page_policy = "close_adaptive"

    # Power related parameters
    # Reference: https://media-www.micron.com/-/media/client/global/
    # documents/products/data-sheet/dram/ddr5/16gb_ddr5_sdram_diereva.pdf
    # Using the values for DDR5-4800
    # DDR5 has one voltage domain, so all the
    # CurrentVariable2 params should not be used or set to 0
    IDD0 = "122mA"
    # Using the value of IDD2P
    IDD2P0 = "88mA"
    IDD2N = "92mA"
    # Using the value of IDD3P
    IDD3P0 = "140mA"
    IDD3N = "142mA"
    IDD4W = "479mA"
    IDD4R = "530mA"
    # IDD5B - 277, IDD5C -- 135mA, IDD5F -- 262mA in the datasheet
    IDD5 = "277mA"
    # IDD6N
    IDD6 = "102mA"

    VDD = "1.1V"


# Maximum bandwidth of DDR5_6400_4x8 (6400 MT/s) can be 25.6GB/s
class DDR5_6400_4x8(DDR5_4400_4x8):
    # For 6400 MT/s
    tCK = "0.312ns"

    # 16 beats across an x32 interface translates to 8 clocks @ 3200 MHz
    tBURST = "2.496ns"

    tRCD = "14.375ns"
    tCL = "14.375ns"
    # tCWL = tCL - 2(nCK)
    tCWL = "13.751ns"
    tRP = "14.375ns"

    # RRD_S (different bank group) : 8nCK
    tRRD = "2.496ns"

    # RRD_L (same bank group) is MAX(8nCK, 5ns) for 1KiB page
    tRRD_L = "5ns"

    # tFAW for 1KiB page is MAX(32 CK, 10.00ns)
    tXAW = "10ns"

    # Rd/Wr turnaround timings
    ###################################################################
    # Note: gem5 adds tBURST separately while calculting rd--rd/wr or
    # wr--wr/rd delays so we can ignore tBURST factor from the following
    # equations while calculating the final value of the timing parameter
    ####################################################################
    # From the datasheet
    # (1) tCCD_L_RTW =
    # CL - CWL + RBL/2 + 2 tCK - (Read DQS offset) + (tRPST - 0.5 tCK) + tWPRE
    # where CWL = CL-2, RBL/2 = tBURST, Read DQS offset = 1ck, tRPST = 1.5tCK
    # Therefore, tCCD_L_RTW =
    # (14.375 - 13.751) + (2*0.312) - 0.312 + ((1.5*0.312)-(0.5*0.312)
    # + (1.5*0.312) = 1.716ns

    # (2) tCCD_S_RTW =
    # CL - CWL + RBL/2 + 2 tCK - (Read DQS offset) + (tRPST - 0.5 tCK) + tWPRE
    # Therefore, tCCD_S_RTW = tCCD_L_RTW = 1.716ns

    # (3) tCCD_L_WTR =
    # CWL + WBL/2 + max(16nCK,10ns)
    # where WBL/2 = tBURST
    # Therefore,
    # tCCD_L_WTR = 13.751+10 = 23.751ns

    # (4) tCCD_S_WTR =
    # CWL + WBL/2 + max(4nCK,2.5ns)
    # where WBL/2 = tBURST
    # Therefore,
    # tCCD_S_WTR = 13.751+2.5 = 16.251ns

    tRTW = "1.716ns"
    tWTR_L = "23.751ns"
    tWTR = "16.251ns"


# Maximum bandwidth of DDR5_8400_4x8 (8400 MT/s) can be 33.6GB/s
# Most of the timing parameters for DDR5_8400_4x8 are TBD in
# the datasheet referred above.
# The TBD parameters are extrapolated from the speed bins mentioned above.
class DDR5_8400_4x8(DDR5_4400_4x8):
    # For 8400 MT/s
    tCK = "0.238ns"

    # 16 beats across an x32 interface translates to 8 clocks @ 4200 MHz
    tBURST = "1.904ns"

    tRCD = "14.285ns"
    tCL = "14.285ns"
    tCWL = "13.809ns"
    tRP = "14.285ns"

    # RRD_S (different bank group) : 8nCK
    tRRD = "1.904ns"

    # tFAW for 1KiB page is MAX(32 CK, 10.00ns)
    tXAW = "10ns"

    # Rd/Wr turnaround timings
    ###################################################################
    # Note: gem5 adds tBURST separately while calculting rd--rd/wr or
    # wr--wr/rd delays so we can ignore tBURST factor from the following
    # equations while calculating the final value of the timing parameter
    ####################################################################
    # From the datasheet
    # (1) tCCD_L_RTW =
    # CL - CWL + RBL/2 + 2 tCK - (Read DQS offset) + (tRPST - 0.5 tCK) + tWPRE
    # where CWL = CL-2, RBL/2 = tBURST, Read DQS offset = 1ck, tRPST = 1.5tCK
    # Therefore, tCCD_L_RTW =
    # (14.285 - 13.809) + (2*0.238) - 0.238 + ((1.5*0.238)-(0.5*0.238)
    # + (1.5*0.238) = 1.309ns

    # (2) tCCD_S_RTW =
    # CL - CWL + RBL/2 + 2 tCK - (Read DQS offset) + (tRPST - 0.5 tCK) + tWPRE
    # Therefore, tCCD_S_RTW = tCCD_L_RTW = 1.309ns

    # (3) tCCD_L_WTR =
    # CWL + WBL/2 + max(16nCK,10ns)
    # where WBL/2 = tBURST
    # Therefore,
    # tCCD_L_WTR =13.809+10 = 23.809ns

    # (4) tCCD_S_WTR =
    # CWL + WBL/2 + max(4nCK,2.5ns)
    # where WBL/2 = tBURST
    # Therefore,
    # tCCD_S_WTR = 13.809+2.5 = 16.309ns

    tRTW = "1.309ns"
    tWTR_L = "23.809ns"
    tWTR = "16.309ns"
