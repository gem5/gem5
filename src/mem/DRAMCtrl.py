# Copyright (c) 2012-2020 ARM Limited
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

from m5.params import *
from m5.proxy import *
from m5.objects.AbstractMemory import *
from m5.objects.QoSMemCtrl import *

# Enum for memory scheduling algorithms, currently First-Come
# First-Served and a First-Row Hit then First-Come First-Served
class MemSched(Enum): vals = ['fcfs', 'frfcfs']

# Enum for the address mapping. With Ch, Ra, Ba, Ro and Co denoting
# channel, rank, bank, row and column, respectively, and going from
# MSB to LSB.  Available are RoRaBaChCo and RoRaBaCoCh, that are
# suitable for an open-page policy, optimising for sequential accesses
# hitting in the open row. For a closed-page policy, RoCoRaBaCh
# maximises parallelism.
class AddrMap(Enum): vals = ['RoRaBaChCo', 'RoRaBaCoCh', 'RoCoRaBaCh']

# Enum for the page policy, either open, open_adaptive, close, or
# close_adaptive.
class PageManage(Enum): vals = ['open', 'open_adaptive', 'close',
                                'close_adaptive']

# DRAMCtrl is a single-channel single-ported DRAM controller model
# that aims to model the most important system-level performance
# effects of a DRAM without getting into too much detail of the DRAM
# itself.
class DRAMCtrl(QoSMemCtrl):
    type = 'DRAMCtrl'
    cxx_header = "mem/dram_ctrl.hh"

    # single-ported on the system interface side, instantiate with a
    # bus in front of the controller for multiple ports
    port = SlavePort("Slave port")

    # the basic configuration of the controller architecture, note
    # that each entry corresponds to a burst for the specific DRAM
    # configuration (e.g. x32 with burst length 8 is 32 bytes) and not
    # the cacheline size or request/packet size
    write_buffer_size = Param.Unsigned(64, "Number of write queue entries")
    read_buffer_size = Param.Unsigned(32, "Number of read queue entries")

    # threshold in percent for when to forcefully trigger writes and
    # start emptying the write buffer
    write_high_thresh_perc = Param.Percent(85, "Threshold to force writes")

    # threshold in percentage for when to start writes if the read
    # queue is empty
    write_low_thresh_perc = Param.Percent(50, "Threshold to start writes")

    # minimum write bursts to schedule before switching back to reads
    min_writes_per_switch = Param.Unsigned(16, "Minimum write bursts before "
                                           "switching to reads")

    # scheduler, address map and page policy
    mem_sched_policy = Param.MemSched('frfcfs', "Memory scheduling policy")
    addr_mapping = Param.AddrMap('RoRaBaCoCh', "Address mapping policy")
    page_policy = Param.PageManage('open_adaptive', "Page management policy")

    # enforce a limit on the number of accesses per row
    max_accesses_per_row = Param.Unsigned(16, "Max accesses per row before "
                                          "closing");

    # size of DRAM Chip in Bytes
    device_size = Param.MemorySize("Size of DRAM chip")

    # pipeline latency of the controller and PHY, split into a
    # frontend part and a backend part, with reads and writes serviced
    # by the queues only seeing the frontend contribution, and reads
    # serviced by the memory seeing the sum of the two
    static_frontend_latency = Param.Latency("10ns", "Static frontend latency")
    static_backend_latency = Param.Latency("10ns", "Static backend latency")

    # the physical organisation of the DRAM
    device_bus_width = Param.Unsigned("data bus width in bits for each DRAM "\
                                      "device/chip")
    burst_length = Param.Unsigned("Burst lenght (BL) in beats")
    device_rowbuffer_size = Param.MemorySize("Page (row buffer) size per "\
                                           "device/chip")
    devices_per_rank = Param.Unsigned("Number of devices/chips per rank")
    ranks_per_channel = Param.Unsigned("Number of ranks per channel")

    # default to 0 bank groups per rank, indicating bank group architecture
    # is not used
    # update per memory class when bank group architecture is supported
    bank_groups_per_rank = Param.Unsigned(0, "Number of bank groups per rank")
    banks_per_rank = Param.Unsigned("Number of banks per rank")

    # Enable DRAM powerdown states if True. This is False by default due to
    # performance being lower when enabled
    enable_dram_powerdown = Param.Bool(False, "Enable powerdown states")

    # For power modelling we need to know if the DRAM has a DLL or not
    dll = Param.Bool(True, "DRAM has DLL or not")

    # DRAMPower provides in addition to the core power, the possibility to
    # include RD/WR termination and IO power. This calculation assumes some
    # default values. The integration of DRAMPower with gem5 does not include
    # IO and RD/WR termination power by default. This might be added as an
    # additional feature in the future.

    # timing behaviour and constraints - all in nanoseconds

    # the base clock period of the DRAM
    tCK = Param.Latency("Clock period")

    # the amount of time in nanoseconds from issuing an activate command
    # to the data being available in the row buffer for a read/write
    tRCD = Param.Latency("RAS to CAS delay")

    # the time from issuing a read/write command to seeing the actual data
    tCL = Param.Latency("CAS latency")

    # minimum time between a precharge and subsequent activate
    tRP = Param.Latency("Row precharge time")

    # minimum time between an activate and a precharge to the same row
    tRAS = Param.Latency("ACT to PRE delay")

    # minimum time between a write data transfer and a precharge
    tWR = Param.Latency("Write recovery time")

    # minimum time between a read and precharge command
    tRTP = Param.Latency("Read to precharge")

    # time to complete a burst transfer, typically the burst length
    # divided by two due to the DDR bus, but by making it a parameter
    # it is easier to also evaluate SDR memories like WideIO.
    # This parameter has to account for burst length.
    # Read/Write requests with data size larger than one full burst are broken
    # down into multiple requests in the controller
    # tBURST is equivalent to the CAS-to-CAS delay (tCCD)
    # With bank group architectures, tBURST represents the CAS-to-CAS
    # delay for bursts to different bank groups (tCCD_S)
    tBURST = Param.Latency("Burst duration "
                           "(typically burst length / 2 cycles)")

    # tBURST_MAX is the column array cycle delay required before next access,
    # which could be greater than tBURST when the memory access time is greater
    # than tBURST
    tBURST_MAX = Param.Latency(Self.tBURST, "Column access delay")

    # tBURST_MIN is the minimum delay between bursts, which could be less than
    # tBURST when interleaving is supported
    tBURST_MIN = Param.Latency(Self.tBURST, "Minimim delay between bursts")

    # CAS-to-CAS delay for bursts to the same bank group
    # only utilized with bank group architectures; set to 0 for default case
    # tBURST is equivalent to tCCD_S; no explicit parameter required
    # for CAS-to-CAS delay for bursts to different bank groups
    tCCD_L = Param.Latency("0ns", "Same bank group CAS to CAS delay")

    # Write-to-Write delay for bursts to the same bank group
    # only utilized with bank group architectures; set to 0 for default case
    # This will be used to enable different same bank group delays
    # for writes versus reads
    tCCD_L_WR = Param.Latency(Self.tCCD_L,
        "Same bank group Write to Write delay")

    # time taken to complete one refresh cycle (N rows in all banks)
    tRFC = Param.Latency("Refresh cycle time")

    # refresh command interval, how often a "ref" command needs
    # to be sent. It is 7.8 us for a 64ms refresh requirement
    tREFI = Param.Latency("Refresh command interval")

    # write-to-read, same rank turnaround penalty
    tWTR = Param.Latency("Write to read, same rank switching time")

    # write-to-read, same rank turnaround penalty for same bank group
    tWTR_L = Param.Latency(Self.tWTR, "Write to read, same rank switching "
                           "time, same bank group")

    # read-to-write, same rank turnaround penalty
    tRTW = Param.Latency("Read to write, same rank switching time")

    # rank-to-rank bus delay penalty
    # this does not correlate to a memory timing parameter and encompasses:
    # 1) RD-to-RD, 2) WR-to-WR, 3) RD-to-WR, and 4) WR-to-RD
    # different rank bus delay
    tCS = Param.Latency("Rank to rank switching time")

    # minimum precharge to precharge delay time
    tPPD = Param.Latency("0ns", "PRE to PRE delay")

    # maximum delay between two-cycle ACT command phases
    tAAD = Param.Latency(Self.tCK,
                         "Maximum delay between two-cycle ACT commands")

    two_cycle_activate = Param.Bool(False,
                         "Two cycles required to send activate")

    # minimum row activate to row activate delay time
    tRRD = Param.Latency("ACT to ACT delay")

    # only utilized with bank group architectures; set to 0 for default case
    tRRD_L = Param.Latency("0ns", "Same bank group ACT to ACT delay")

    # time window in which a maximum number of activates are allowed
    # to take place, set to 0 to disable
    tXAW = Param.Latency("X activation window")
    activation_limit = Param.Unsigned("Max number of activates in window")

    # time to exit power-down mode
    # Exit power-down to next valid command delay
    tXP = Param.Latency("0ns", "Power-up Delay")

    # Exit Powerdown to commands requiring a locked DLL
    tXPDLL = Param.Latency("0ns", "Power-up Delay with locked DLL")

    # time to exit self-refresh mode
    tXS = Param.Latency("0ns", "Self-refresh exit latency")

    # time to exit self-refresh mode with locked DLL
    tXSDLL = Param.Latency("0ns", "Self-refresh exit latency DLL")

    # number of data beats per clock. with DDR, default is 2, one per edge
    beats_per_clock = Param.Unsigned(2, "Data beats per clock")

    data_clock_sync = Param.Bool(False, "Synchronization commands required")

    # Currently rolled into other params
    ######################################################################

    # tRC  - assumed to be tRAS + tRP

    # Power Behaviour and Constraints
    # DRAMs like LPDDR and WideIO have 2 external voltage domains. These are
    # defined as VDD and VDD2. Each current is defined for each voltage domain
    # separately. For example, current IDD0 is active-precharge current for
    # voltage domain VDD and current IDD02 is active-precharge current for
    # voltage domain VDD2.
    # By default all currents are set to 0mA. Users who are only interested in
    # the performance of DRAMs can leave them at 0.

    # Operating 1 Bank Active-Precharge current
    IDD0 = Param.Current("0mA", "Active precharge current")

    # Operating 1 Bank Active-Precharge current multiple voltage Range
    IDD02 = Param.Current("0mA", "Active precharge current VDD2")

    # Precharge Power-down Current: Slow exit
    IDD2P0 = Param.Current("0mA", "Precharge Powerdown slow")

    # Precharge Power-down Current: Slow exit multiple voltage Range
    IDD2P02 = Param.Current("0mA", "Precharge Powerdown slow VDD2")

    # Precharge Power-down Current: Fast exit
    IDD2P1 = Param.Current("0mA", "Precharge Powerdown fast")

    # Precharge Power-down Current: Fast exit multiple voltage Range
    IDD2P12 = Param.Current("0mA", "Precharge Powerdown fast VDD2")

    # Precharge Standby current
    IDD2N = Param.Current("0mA", "Precharge Standby current")

    # Precharge Standby current multiple voltage range
    IDD2N2 = Param.Current("0mA", "Precharge Standby current VDD2")

    # Active Power-down current: slow exit
    IDD3P0 = Param.Current("0mA", "Active Powerdown slow")

    # Active Power-down current: slow exit multiple voltage range
    IDD3P02 = Param.Current("0mA", "Active Powerdown slow VDD2")

    # Active Power-down current : fast exit
    IDD3P1 = Param.Current("0mA", "Active Powerdown fast")

    # Active Power-down current : fast exit multiple voltage range
    IDD3P12 = Param.Current("0mA", "Active Powerdown fast VDD2")

    # Active Standby current
    IDD3N = Param.Current("0mA", "Active Standby current")

    # Active Standby current multiple voltage range
    IDD3N2 = Param.Current("0mA", "Active Standby current VDD2")

    # Burst Read Operating Current
    IDD4R = Param.Current("0mA", "READ current")

    # Burst Read Operating Current multiple voltage range
    IDD4R2 = Param.Current("0mA", "READ current VDD2")

    # Burst Write Operating Current
    IDD4W = Param.Current("0mA", "WRITE current")

    # Burst Write Operating Current multiple voltage range
    IDD4W2 = Param.Current("0mA", "WRITE current VDD2")

    # Refresh Current
    IDD5 = Param.Current("0mA", "Refresh current")

    # Refresh Current multiple voltage range
    IDD52 = Param.Current("0mA", "Refresh current VDD2")

    # Self-Refresh Current
    IDD6 = Param.Current("0mA", "Self-refresh Current")

    # Self-Refresh Current multiple voltage range
    IDD62 = Param.Current("0mA", "Self-refresh Current VDD2")

    # Main voltage range of the DRAM
    VDD = Param.Voltage("0V", "Main Voltage Range")

    # Second voltage range defined by some DRAMs
    VDD2 = Param.Voltage("0V", "2nd Voltage Range")

# A single DDR3-1600 x64 channel (one command and address bus), with
# timings based on a DDR3-1600 4 Gbit datasheet (Micron MT41J512M8) in
# an 8x8 configuration.
class DDR3_1600_8x8(DRAMCtrl):
    # size of device in bytes
    device_size = '512MB'

    # 8x8 configuration, 8 devices each with an 8-bit interface
    device_bus_width = 8

    # DDR3 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 1 Kbyte (1K columns x8)
    device_rowbuffer_size = '1kB'

    # 8x8 configuration, so 8 devices
    devices_per_rank = 8

    # Use two ranks
    ranks_per_channel = 2

    # DDR3 has 8 banks in all configurations
    banks_per_rank = 8

    # 800 MHz
    tCK = '1.25ns'

    # 8 beats across an x64 interface translates to 4 clocks @ 800 MHz
    tBURST = '5ns'

    # DDR3-1600 11-11-11
    tRCD = '13.75ns'
    tCL = '13.75ns'
    tRP = '13.75ns'
    tRAS = '35ns'
    tRRD = '6ns'
    tXAW = '30ns'
    activation_limit = 4
    tRFC = '260ns'

    tWR = '15ns'

    # Greater of 4 CK or 7.5 ns
    tWTR = '7.5ns'

    # Greater of 4 CK or 7.5 ns
    tRTP = '7.5ns'

    # Default same rank rd-to-wr bus turnaround to 2 CK, @800 MHz = 2.5 ns
    tRTW = '2.5ns'

    # Default different rank bus delay to 2 CK, @800 MHz = 2.5 ns
    tCS = '2.5ns'

    # <=85C, half for >85C
    tREFI = '7.8us'

    # active powerdown and precharge powerdown exit time
    tXP = '6ns'

    # self refresh exit time
    tXS = '270ns'

    # Current values from datasheet Die Rev E,J
    IDD0 = '55mA'
    IDD2N = '32mA'
    IDD3N = '38mA'
    IDD4W = '125mA'
    IDD4R = '157mA'
    IDD5 = '235mA'
    IDD3P1 = '38mA'
    IDD2P1 = '32mA'
    IDD6 = '20mA'
    VDD = '1.5V'

# A single HMC-2500 x32 model based on:
# [1] DRAMSpec: a high-level DRAM bank modelling tool
# developed at the University of Kaiserslautern. This high level tool
# uses RC (resistance-capacitance) and CV (capacitance-voltage) models to
# estimate the DRAM bank latency and power numbers.
# [2] High performance AXI-4.0 based interconnect for extensible smart memory
# cubes (E. Azarkhish et. al)
# Assumed for the HMC model is a 30 nm technology node.
# The modelled HMC consists of 4 Gbit layers which sum up to 2GB of memory (4
# layers).
# Each layer has 16 vaults and each vault consists of 2 banks per layer.
# In order to be able to use the same controller used for 2D DRAM generations
# for HMC, the following analogy is done:
# Channel (DDR) => Vault (HMC)
# device_size (DDR) => size of a single layer in a vault
# ranks per channel (DDR) => number of layers
# banks per rank (DDR) => banks per layer
# devices per rank (DDR) => devices per layer ( 1 for HMC).
# The parameters for which no input is available are inherited from the DDR3
# configuration.
# This configuration includes the latencies from the DRAM to the logic layer
# of the HMC
class HMC_2500_1x32(DDR3_1600_8x8):
    # size of device
    # two banks per device with each bank 4MB [2]
    device_size = '8MB'

    # 1x32 configuration, 1 device with 32 TSVs [2]
    device_bus_width = 32

    # HMC is a BL8 device [2]
    burst_length = 8

    # Each device has a page (row buffer) size of 256 bytes [2]
    device_rowbuffer_size = '256B'

    # 1x32 configuration, so 1 device [2]
    devices_per_rank = 1

    # 4 layers so 4 ranks [2]
    ranks_per_channel = 4

    # HMC has 2 banks per layer [2]
    # Each layer represents a rank. With 4 layers and 8 banks in total, each
    # layer has 2 banks; thus 2 banks per rank.
    banks_per_rank = 2

    # 1250 MHz [2]
    tCK = '0.8ns'

    # 8 beats across an x32 interface translates to 4 clocks @ 1250 MHz
    tBURST = '3.2ns'

    # Values using DRAMSpec HMC model [1]
    tRCD = '10.2ns'
    tCL = '9.9ns'
    tRP = '7.7ns'
    tRAS = '21.6ns'

    # tRRD depends on the power supply network for each vendor.
    # We assume a tRRD of a double bank approach to be equal to 4 clock
    # cycles (Assumption)
    tRRD = '3.2ns'

    # activation limit is set to 0 since there are only 2 banks per vault
    # layer.
    activation_limit = 0

    # Values using DRAMSpec HMC model [1]
    tRFC = '59ns'
    tWR = '8ns'
    tRTP = '4.9ns'

    # Default different rank bus delay assumed to 1 CK for TSVs, @1250 MHz =
    # 0.8 ns (Assumption)
    tCS = '0.8ns'

    # Value using DRAMSpec HMC model [1]
    tREFI = '3.9us'

    # The default page policy in the vault controllers is simple closed page
    # [2] nevertheless 'close' policy opens and closes the row multiple times
    # for bursts largers than 32Bytes. For this reason we use 'close_adaptive'
    page_policy = 'close_adaptive'

    # RoCoRaBaCh resembles the default address mapping in HMC
    addr_mapping = 'RoCoRaBaCh'
    min_writes_per_switch = 8

    # These parameters do not directly correlate with buffer_size in real
    # hardware. Nevertheless, their value has been tuned to achieve a
    # bandwidth similar to the cycle-accurate model in [2]
    write_buffer_size = 32
    read_buffer_size = 32

    # The static latency of the vault controllers is estimated to be smaller
    # than a full DRAM channel controller
    static_backend_latency='4ns'
    static_frontend_latency='4ns'

# A single DDR3-2133 x64 channel refining a selected subset of the
# options for the DDR-1600 configuration, based on the same DDR3-1600
# 4 Gbit datasheet (Micron MT41J512M8). Most parameters are kept
# consistent across the two configurations.
class DDR3_2133_8x8(DDR3_1600_8x8):
    # 1066 MHz
    tCK = '0.938ns'

    # 8 beats across an x64 interface translates to 4 clocks @ 1066 MHz
    tBURST = '3.752ns'

    # DDR3-2133 14-14-14
    tRCD = '13.09ns'
    tCL = '13.09ns'
    tRP = '13.09ns'
    tRAS = '33ns'
    tRRD = '5ns'
    tXAW = '25ns'

    # Current values from datasheet
    IDD0 = '70mA'
    IDD2N = '37mA'
    IDD3N = '44mA'
    IDD4W = '157mA'
    IDD4R = '191mA'
    IDD5 = '250mA'
    IDD3P1 = '44mA'
    IDD2P1 = '43mA'
    IDD6 ='20mA'
    VDD = '1.5V'

# A single DDR4-2400 x64 channel (one command and address bus), with
# timings based on a DDR4-2400 8 Gbit datasheet (Micron MT40A2G4)
# in an 16x4 configuration.
# Total channel capacity is 32GB
# 16 devices/rank * 2 ranks/channel * 1GB/device = 32GB/channel
class DDR4_2400_16x4(DRAMCtrl):
    # size of device
    device_size = '1GB'

    # 16x4 configuration, 16 devices each with a 4-bit interface
    device_bus_width = 4

    # DDR4 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 512 byte (1K columns x4)
    device_rowbuffer_size = '512B'

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
    tCK = '0.833ns'

    # 8 beats across an x64 interface translates to 4 clocks @ 1200 MHz
    # tBURST is equivalent to the CAS-to-CAS delay (tCCD)
    # With bank group architectures, tBURST represents the CAS-to-CAS
    # delay for bursts to different bank groups (tCCD_S)
    tBURST = '3.332ns'

    # @2400 data rate, tCCD_L is 6 CK
    # CAS-to-CAS delay for bursts to the same bank group
    # tBURST is equivalent to tCCD_S; no explicit parameter required
    # for CAS-to-CAS delay for bursts to different bank groups
    tCCD_L = '5ns';

    # DDR4-2400 17-17-17
    tRCD = '14.16ns'
    tCL = '14.16ns'
    tRP = '14.16ns'
    tRAS = '32ns'

    # RRD_S (different bank group) for 512B page is MAX(4 CK, 3.3ns)
    tRRD = '3.332ns'

    # RRD_L (same bank group) for 512B page is MAX(4 CK, 4.9ns)
    tRRD_L = '4.9ns';

    # tFAW for 512B page is MAX(16 CK, 13ns)
    tXAW = '13.328ns'
    activation_limit = 4
    # tRFC is 350ns
    tRFC = '350ns'

    tWR = '15ns'

    # Here using the average of WTR_S and WTR_L
    tWTR = '5ns'

    # Greater of 4 CK or 7.5 ns
    tRTP = '7.5ns'

    # Default same rank rd-to-wr bus turnaround to 2 CK, @1200 MHz = 1.666 ns
    tRTW = '1.666ns'

    # Default different rank bus delay to 2 CK, @1200 MHz = 1.666 ns
    tCS = '1.666ns'

    # <=85C, half for >85C
    tREFI = '7.8us'

    # active powerdown and precharge powerdown exit time
    tXP = '6ns'

    # self refresh exit time
    # exit delay to ACT, PRE, PREALL, REF, SREF Enter, and PD Enter is:
    # tRFC + 10ns = 340ns
    tXS = '340ns'

    # Current values from datasheet
    IDD0 = '43mA'
    IDD02 = '3mA'
    IDD2N = '34mA'
    IDD3N = '38mA'
    IDD3N2 = '3mA'
    IDD4W = '103mA'
    IDD4R = '110mA'
    IDD5 = '250mA'
    IDD3P1 = '32mA'
    IDD2P1 = '25mA'
    IDD6 = '30mA'
    VDD = '1.2V'
    VDD2 = '2.5V'

# A single DDR4-2400 x64 channel (one command and address bus), with
# timings based on a DDR4-2400 8 Gbit datasheet (Micron MT40A1G8)
# in an 8x8 configuration.
# Total channel capacity is 16GB
# 8 devices/rank * 2 ranks/channel * 1GB/device = 16GB/channel
class DDR4_2400_8x8(DDR4_2400_16x4):
    # 8x8 configuration, 8 devices each with an 8-bit interface
    device_bus_width = 8

    # Each device has a page (row buffer) size of 1 Kbyte (1K columns x8)
    device_rowbuffer_size = '1kB'

    # 8x8 configuration, so 8 devices
    devices_per_rank = 8

    # RRD_L (same bank group) for 1K page is MAX(4 CK, 4.9ns)
    tRRD_L = '4.9ns';

    tXAW = '21ns'

    # Current values from datasheet
    IDD0 = '48mA'
    IDD3N = '43mA'
    IDD4W = '123mA'
    IDD4R = '135mA'
    IDD3P1 = '37mA'

# A single DDR4-2400 x64 channel (one command and address bus), with
# timings based on a DDR4-2400 8 Gbit datasheet (Micron MT40A512M16)
# in an 4x16 configuration.
# Total channel capacity is 4GB
# 4 devices/rank * 1 ranks/channel * 1GB/device = 4GB/channel
class DDR4_2400_4x16(DDR4_2400_16x4):
    # 4x16 configuration, 4 devices each with an 16-bit interface
    device_bus_width = 16

    # Each device has a page (row buffer) size of 2 Kbyte (1K columns x16)
    device_rowbuffer_size = '2kB'

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
    tRRD = '5.3ns'

    # RRD_L (same bank group) for 2K page is MAX(4 CK, 6.4ns)
    tRRD_L = '6.4ns';

    tXAW = '30ns'

    # Current values from datasheet
    IDD0 = '80mA'
    IDD02 = '4mA'
    IDD2N = '34mA'
    IDD3N = '47mA'
    IDD4W = '228mA'
    IDD4R = '243mA'
    IDD5 = '280mA'
    IDD3P1 = '41mA'

# A single LPDDR2-S4 x32 interface (one command/address bus), with
# default timings based on a LPDDR2-1066 4 Gbit part (Micron MT42L128M32D1)
# in a 1x32 configuration.
class LPDDR2_S4_1066_1x32(DRAMCtrl):
    # No DLL in LPDDR2
    dll = False

    # size of device
    device_size = '512MB'

    # 1x32 configuration, 1 device with a 32-bit interface
    device_bus_width = 32

    # LPDDR2_S4 is a BL4 and BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 1KB
    # (this depends on the memory density)
    device_rowbuffer_size = '1kB'

    # 1x32 configuration, so 1 device
    devices_per_rank = 1

    # Use a single rank
    ranks_per_channel = 1

    # LPDDR2-S4 has 8 banks in all configurations
    banks_per_rank = 8

    # 533 MHz
    tCK = '1.876ns'

    # Fixed at 15 ns
    tRCD = '15ns'

    # 8 CK read latency, 4 CK write latency @ 533 MHz, 1.876 ns cycle time
    tCL = '15ns'

    # Pre-charge one bank 15 ns (all banks 18 ns)
    tRP = '15ns'

    tRAS = '42ns'
    tWR = '15ns'

    tRTP = '7.5ns'

    # 8 beats across an x32 DDR interface translates to 4 clocks @ 533 MHz.
    # Note this is a BL8 DDR device.
    # Requests larger than 32 bytes are broken down into multiple requests
    # in the controller
    tBURST = '7.5ns'

    # LPDDR2-S4, 4 Gbit
    tRFC = '130ns'
    tREFI = '3.9us'

    # active powerdown and precharge powerdown exit time
    tXP = '7.5ns'

    # self refresh exit time
    tXS = '140ns'

    # Irrespective of speed grade, tWTR is 7.5 ns
    tWTR = '7.5ns'

    # Default same rank rd-to-wr bus turnaround to 2 CK, @533 MHz = 3.75 ns
    tRTW = '3.75ns'

    # Default different rank bus delay to 2 CK, @533 MHz = 3.75 ns
    tCS = '3.75ns'

    # Activate to activate irrespective of density and speed grade
    tRRD = '10.0ns'

    # Irrespective of density, tFAW is 50 ns
    tXAW = '50ns'
    activation_limit = 4

    # Current values from datasheet
    IDD0 = '15mA'
    IDD02 = '70mA'
    IDD2N = '2mA'
    IDD2N2 = '30mA'
    IDD3N = '2.5mA'
    IDD3N2 = '30mA'
    IDD4W = '10mA'
    IDD4W2 = '190mA'
    IDD4R = '3mA'
    IDD4R2 = '220mA'
    IDD5 = '40mA'
    IDD52 = '150mA'
    IDD3P1 = '1.2mA'
    IDD3P12 = '8mA'
    IDD2P1 = '0.6mA'
    IDD2P12 = '0.8mA'
    IDD6 = '1mA'
    IDD62 = '3.2mA'
    VDD = '1.8V'
    VDD2 = '1.2V'

# A single WideIO x128 interface (one command and address bus), with
# default timings based on an estimated WIO-200 8 Gbit part.
class WideIO_200_1x128(DRAMCtrl):
    # No DLL for WideIO
    dll = False

    # size of device
    device_size = '1024MB'

    # 1x128 configuration, 1 device with a 128-bit interface
    device_bus_width = 128

    # This is a BL4 device
    burst_length = 4

    # Each device has a page (row buffer) size of 4KB
    # (this depends on the memory density)
    device_rowbuffer_size = '4kB'

    # 1x128 configuration, so 1 device
    devices_per_rank = 1

    # Use one rank for a one-high die stack
    ranks_per_channel = 1

    # WideIO has 4 banks in all configurations
    banks_per_rank = 4

    # 200 MHz
    tCK = '5ns'

    # WIO-200
    tRCD = '18ns'
    tCL = '18ns'
    tRP = '18ns'
    tRAS = '42ns'
    tWR = '15ns'
    # Read to precharge is same as the burst
    tRTP = '20ns'

    # 4 beats across an x128 SDR interface translates to 4 clocks @ 200 MHz.
    # Note this is a BL4 SDR device.
    tBURST = '20ns'

    # WIO 8 Gb
    tRFC = '210ns'

    # WIO 8 Gb, <=85C, half for >85C
    tREFI = '3.9us'

    # Greater of 2 CK or 15 ns, 2 CK @ 200 MHz = 10 ns
    tWTR = '15ns'

    # Default same rank rd-to-wr bus turnaround to 2 CK, @200 MHz = 10 ns
    tRTW = '10ns'

    # Default different rank bus delay to 2 CK, @200 MHz = 10 ns
    tCS = '10ns'

    # Activate to activate irrespective of density and speed grade
    tRRD = '10.0ns'

    # Two instead of four activation window
    tXAW = '50ns'
    activation_limit = 2

    # The WideIO specification does not provide current information

# A single LPDDR3 x32 interface (one command/address bus), with
# default timings based on a LPDDR3-1600 4 Gbit part (Micron
# EDF8132A1MC) in a 1x32 configuration.
class LPDDR3_1600_1x32(DRAMCtrl):
    # No DLL for LPDDR3
    dll = False

    # size of device
    device_size = '512MB'

    # 1x32 configuration, 1 device with a 32-bit interface
    device_bus_width = 32

    # LPDDR3 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 4KB
    device_rowbuffer_size = '4kB'

    # 1x32 configuration, so 1 device
    devices_per_rank = 1

    # Technically the datasheet is a dual-rank package, but for
    # comparison with the LPDDR2 config we stick to a single rank
    ranks_per_channel = 1

    # LPDDR3 has 8 banks in all configurations
    banks_per_rank = 8

    # 800 MHz
    tCK = '1.25ns'

    tRCD = '18ns'

    # 12 CK read latency, 6 CK write latency @ 800 MHz, 1.25 ns cycle time
    tCL = '15ns'

    tRAS = '42ns'
    tWR = '15ns'

    # Greater of 4 CK or 7.5 ns, 4 CK @ 800 MHz = 5 ns
    tRTP = '7.5ns'

    # Pre-charge one bank 18 ns (all banks 21 ns)
    tRP = '18ns'

    # 8 beats across a x32 DDR interface translates to 4 clocks @ 800 MHz.
    # Note this is a BL8 DDR device.
    # Requests larger than 32 bytes are broken down into multiple requests
    # in the controller
    tBURST = '5ns'

    # LPDDR3, 4 Gb
    tRFC = '130ns'
    tREFI = '3.9us'

    # active powerdown and precharge powerdown exit time
    tXP = '7.5ns'

    # self refresh exit time
    tXS = '140ns'

    # Irrespective of speed grade, tWTR is 7.5 ns
    tWTR = '7.5ns'

    # Default same rank rd-to-wr bus turnaround to 2 CK, @800 MHz = 2.5 ns
    tRTW = '2.5ns'

    # Default different rank bus delay to 2 CK, @800 MHz = 2.5 ns
    tCS = '2.5ns'

    # Activate to activate irrespective of density and speed grade
    tRRD = '10.0ns'

    # Irrespective of size, tFAW is 50 ns
    tXAW = '50ns'
    activation_limit = 4

    # Current values from datasheet
    IDD0 = '8mA'
    IDD02 = '60mA'
    IDD2N = '0.8mA'
    IDD2N2 = '26mA'
    IDD3N = '2mA'
    IDD3N2 = '34mA'
    IDD4W = '2mA'
    IDD4W2 = '190mA'
    IDD4R = '2mA'
    IDD4R2 = '230mA'
    IDD5 = '28mA'
    IDD52 = '150mA'
    IDD3P1 = '1.4mA'
    IDD3P12 = '11mA'
    IDD2P1 = '0.8mA'
    IDD2P12 = '1.8mA'
    IDD6 = '0.5mA'
    IDD62 = '1.8mA'
    VDD = '1.8V'
    VDD2 = '1.2V'

# A single GDDR5 x64 interface, with
# default timings based on a GDDR5-4000 1 Gbit part (SK Hynix
# H5GQ1H24AFR) in a 2x32 configuration.
class GDDR5_4000_2x32(DRAMCtrl):
    # size of device
    device_size = '128MB'

    # 2x32 configuration, 1 device with a 32-bit interface
    device_bus_width = 32

    # GDDR5 is a BL8 device
    burst_length = 8

    # Each device has a page (row buffer) size of 2Kbits (256Bytes)
    device_rowbuffer_size = '256B'

    # 2x32 configuration, so 2 devices
    devices_per_rank = 2

    # assume single rank
    ranks_per_channel = 1

    # GDDR5 has 4 bank groups
    bank_groups_per_rank = 4

    # GDDR5 has 16 banks with 4 bank groups
    banks_per_rank = 16

    # 1000 MHz
    tCK = '1ns'

    # 8 beats across an x64 interface translates to 2 clocks @ 1000 MHz
    # Data bus runs @2000 Mhz => DDR ( data runs at 4000 MHz )
    # 8 beats at 4000 MHz = 2 beats at 1000 MHz
    # tBURST is equivalent to the CAS-to-CAS delay (tCCD)
    # With bank group architectures, tBURST represents the CAS-to-CAS
    # delay for bursts to different bank groups (tCCD_S)
    tBURST = '2ns'

    # @1000MHz data rate, tCCD_L is 3 CK
    # CAS-to-CAS delay for bursts to the same bank group
    # tBURST is equivalent to tCCD_S; no explicit parameter required
    # for CAS-to-CAS delay for bursts to different bank groups
    tCCD_L = '3ns';

    tRCD = '12ns'

    # tCL is not directly found in datasheet and assumed equal tRCD
    tCL = '12ns'

    tRP = '12ns'
    tRAS = '28ns'

    # RRD_S (different bank group)
    # RRD_S is 5.5 ns in datasheet.
    # rounded to the next multiple of tCK
    tRRD = '6ns'

    # RRD_L (same bank group)
    # RRD_L is 5.5 ns in datasheet.
    # rounded to the next multiple of tCK
    tRRD_L = '6ns'

    tXAW = '23ns'

    # tXAW < 4 x tRRD.
    # Therefore, activation limit is set to 0
    activation_limit = 0

    tRFC = '65ns'
    tWR = '12ns'

    # Here using the average of WTR_S and WTR_L
    tWTR = '5ns'

    # Read-to-Precharge 2 CK
    tRTP = '2ns'

    # Assume 2 cycles
    tRTW = '2ns'

# A single HBM x128 interface (one command and address bus), with
# default timings based on data publically released
# ("HBM: Memory Solution for High Performance Processors", MemCon, 2014),
# IDD measurement values, and by extrapolating data from other classes.
# Architecture values based on published HBM spec
# A 4H stack is defined, 2Gb per die for a total of 1GB of memory.
class HBM_1000_4H_1x128(DRAMCtrl):
    # HBM gen1 supports up to 8 128-bit physical channels
    # Configuration defines a single channel, with the capacity
    # set to (full_ stack_capacity / 8) based on 2Gb dies
    # To use all 8 channels, set 'channels' parameter to 8 in
    # system configuration

    # 128-bit interface legacy mode
    device_bus_width = 128

    # HBM supports BL4 and BL2 (legacy mode only)
    burst_length = 4

    # size of channel in bytes, 4H stack of 2Gb dies is 1GB per stack;
    # with 8 channels, 128MB per channel
    device_size = '128MB'

    device_rowbuffer_size = '2kB'

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
    tCK = '2ns'

    # use values from IDD measurement in JEDEC spec
    # use tRP value for tRCD and tCL similar to other classes
    tRP = '15ns'
    tRCD = '15ns'
    tCL = '15ns'
    tRAS = '33ns'

    # BL2 and BL4 supported, default to BL4
    # DDR @ 500 MHz means 4 * 2ns / 2 = 4ns
    tBURST = '4ns'

    # value for 2Gb device from JEDEC spec
    tRFC = '160ns'

    # value for 2Gb device from JEDEC spec
    tREFI = '3.9us'

    # extrapolate the following from LPDDR configs, using ns values
    # to minimize burst length, prefetch differences
    tWR = '18ns'
    tRTP = '7.5ns'
    tWTR = '10ns'

    # start with 2 cycles turnaround, similar to other memory classes
    # could be more with variations across the stack
    tRTW = '4ns'

    # single rank device, set to 0
    tCS = '0ns'

    # from MemCon example, tRRD is 4ns with 2ns tCK
    tRRD = '4ns'

    # from MemCon example, tFAW is 30ns with 2ns tCK
    tXAW = '30ns'
    activation_limit = 4

    # 4tCK
    tXP = '8ns'

    # start with tRFC + tXP -> 160ns + 8ns = 168ns
    tXS = '168ns'

# A single HBM x64 interface (one command and address bus), with
# default timings based on HBM gen1 and data publically released
# A 4H stack is defined, 8Gb per die for a total of 4GB of memory.
# Note: This defines a pseudo-channel with a unique controller
# instantiated per pseudo-channel
# Stay at same IO rate (1Gbps) to maintain timing relationship with
# HBM gen1 class (HBM_1000_4H_x128) where possible
class HBM_1000_4H_1x64(HBM_1000_4H_1x128):
    # For HBM gen2 with pseudo-channel mode, configure 2X channels.
    # Configuration defines a single pseudo channel, with the capacity
    # set to (full_ stack_capacity / 16) based on 8Gb dies
    # To use all 16 pseudo channels, set 'channels' parameter to 16 in
    # system configuration

    # 64-bit pseudo-channle interface
    device_bus_width = 64

    # HBM pseudo-channel only supports BL4
    burst_length = 4

    # size of channel in bytes, 4H stack of 8Gb dies is 4GB per stack;
    # with 16 channels, 256MB per channel
    device_size = '256MB'

    # page size is halved with pseudo-channel; maintaining the same same number
    # of rows per pseudo-channel with 2X banks across 2 channels
    device_rowbuffer_size = '1kB'

    # HBM has 8 or 16 banks depending on capacity
    # Starting with 4Gb dies, 16 banks are defined
    banks_per_rank = 16

    # reset tRFC for larger, 8Gb device
    # use HBM1 4Gb value as a starting point
    tRFC = '260ns'

    # start with tRFC + tXP -> 160ns + 8ns = 168ns
    tXS = '268ns'
    # Default different rank bus delay to 2 CK, @1000 MHz = 2 ns
    tCS = '2ns'
    tREFI = '3.9us'

    # active powerdown and precharge powerdown exit time
    tXP = '10ns'

    # self refresh exit time
    tXS = '65ns'

# A single LPDDR5 x16 interface (one command/address bus)
# for a single x16 channel with default timings based on
# initial JEDEC specification
# Starting with 5.5Gbps data rates and 8Gbit die
# Configuring for 16-bank mode with bank-group architecture
# burst of 32, which means bursts can be interleaved
class LPDDR5_5500_1x16_BG_BL32(DRAMCtrl):

    # Increase buffer size to account for more bank resources
    read_buffer_size = 64

    # Set page policy to better suit DMC Huxley
    page_policy = 'close_adaptive'

    # 16-bit channel interface
    device_bus_width = 16

    # LPDDR5 is a BL16 or BL32 device
    # With BG mode, BL16 and BL32 are supported
    # Use BL32 for higher command bandwidth
    burst_length = 32

    # size of device in bytes
    device_size = '1GB'

    # 2kB page with BG mode
    device_rowbuffer_size = '2kB'

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
    tCK = '1.455ns'

    # Greater of 2 CK or 18ns
    tRCD = '18ns'

    # Base RL is 16 CK @ 687.5 MHz = 23.28ns
    tCL = '23.280ns'

    # Greater of 2 CK or 18ns
    tRP = '18ns'

    # Greater of 3 CK or 42ns
    tRAS = '42ns'

    # Greater of 3 CK or 34ns
    tWR = '34ns'

    # active powerdown and precharge powerdown exit time
    # Greater of 3 CK or 7ns
    tXP = '7ns'

    # self refresh exit time (tRFCab + 7.5ns)
    tXS = '217.5ns'

    # Greater of 2 CK or 7.5 ns minus 2 CK
    tRTP = '4.59ns'

    # With BG architecture, burst of 32 transferred in two 16-beat
    # sub-bursts, with a 16-beat gap in between.
    # Each 16-beat sub-burst is 8 WCK @2.75 GHz or 2 CK @ 687.5 MHz
    # tBURST is the delay to transfer the Bstof32 =  6 CK @ 687.5 MHz
    tBURST = '8.73ns'
    # can interleave a Bstof32 from another bank group at tBURST_MIN
    # 16-beats is 8 WCK @2.75 GHz or 2 CK @ 687.5 MHz
    tBURST_MIN = '2.91ns'
    # tBURST_MAX is the maximum burst delay for same bank group timing
    # this is 8 CK @ 687.5 MHz
    tBURST_MAX = '11.64ns'

    # 8 CK @ 687.5 MHz
    tCCD_L = "11.64ns"

    # LPDDR5, 8 Gbit/channel for 280ns tRFCab
    tRFC = '210ns'
    tREFI = '3.9us'

    # Greater of 4 CK or 6.25 ns
    tWTR = '6.25ns'
    # Greater of 4 CK or 12 ns
    tWTR_L = '12ns'

    # Required RD-to-WR timing is RL+ BL/n + tWCKDQ0/tCK - WL
    # tWCKDQ0/tCK will be 1 CK for most cases
    # For gem5 RL = WL and BL/n is already accounted for with tBURST
    # Result is and additional 1 CK is required
    tRTW = '1.455ns'

    # Default different rank bus delay to 2 CK, @687.5 MHz = 2.91 ns
    tCS = '2.91ns'

    # 2 CK
    tPPD = '2.91ns'

    # Greater of 2 CK or 5 ns
    tRRD = '5ns'
    tRRD_L = '5ns'

    # With Bank Group Arch mode tFAW is 20 ns
    tXAW = '20ns'
    activation_limit = 4

    # at 5Gbps, 4:1 WCK to CK ratio required
    # 2 data beats per WCK (DDR) -> 8 per CK
    beats_per_clock = 8

    # 2 cycles required to send activate command
    # 2 command phases can be sent back-to-back or
    # with a gap up to tAAD = 8 CK
    two_cycle_activate = True
    tAAD = '11.640ns'

    data_clock_sync = True

# A single LPDDR5 x16 interface (one command/address bus)
# for a single x16 channel with default timings based on
# initial JEDEC specification
# Starting with 5.5Gbps data rates and 8Gbit die
# Configuring for 16-bank mode with bank-group architecture, burst of 16
class LPDDR5_5500_1x16_BG_BL16(LPDDR5_5500_1x16_BG_BL32):

    # LPDDR5 is a BL16 or BL32 device
    # With BG mode, BL16 and BL32 are supported
    # Use BL16 for smaller access granularity
    burst_length = 16

    # For Bstof16 with BG arch, 2 CK @ 687.5 MHz with 4:1 clock ratio
    tBURST = '2.91ns'
    tBURST_MIN = '2.91ns'
    # For Bstof16 with BG arch, 4 CK @ 687.5 MHz with 4:1 clock ratio
    tBURST_MAX = '5.82ns'

    # 4 CK @ 687.5 MHz
    tCCD_L = "5.82ns"


# A single LPDDR5 x16 interface (one command/address bus)
# for a single x16 channel with default timings based on
# initial JEDEC specification
# Starting with 5.5Gbps data rates and 8Gbit die
# Configuring for 8-bank mode, burst of 32
class LPDDR5_5500_1x16_8B_BL32(LPDDR5_5500_1x16_BG_BL32):

    # 4kB page with 8B mode
    device_rowbuffer_size = '4kB'

    # LPDDR5 supports configurable bank options
    # 8B  : BL32, all frequencies
    # 16B : BL32 or BL16, <=3.2Gbps
    # 16B with Bank Group Arch (4B/BG): BL32 or BL16, >3.2Gbps
    # Select 8B
    banks_per_rank = 8
    bank_groups_per_rank = 0

    # For Bstof32 with 8B mode, 4 CK @ 687.5 MHz with 4:1 clock ratio
    tBURST = '5.82ns'
    tBURST_MIN = '5.82ns'
    tBURST_MAX = '5.82ns'

    # Greater of 4 CK or 12 ns
    tWTR = '12ns'

    # Greater of 2 CK or 10 ns
    tRRD = '10ns'

    # With 8B mode tFAW is 40 ns
    tXAW = '40ns'
    activation_limit = 4

    # Reset BG arch timing for 8B mode
    tCCD_L = "0ns"
    tRRD_L = "0ns"
    tWTR_L = "0ns"

# A single LPDDR5 x16 interface (one command/address bus)
# for a single x16 channel with default timings based on
# initial JEDEC specification
# 6.4Gbps data rates and 8Gbit die
# Configuring for 16-bank mode with bank-group architecture
# burst of 32, which means bursts can be interleaved
class LPDDR5_6400_1x16_BG_BL32(LPDDR5_5500_1x16_BG_BL32):

    # 5.5Gb/s DDR with 4:1 WCK:CK ratio for 687.5 MHz CK
    tCK = '1.25ns'

    # Base RL is 17 CK @ 800 MHz = 21.25ns
    tCL = '21.25ns'

    # With BG architecture, burst of 32 transferred in two 16-beat
    # sub-bursts, with a 16-beat gap in between.
    # Each 16-beat sub-burst is 8 WCK @3.2 GHz or 2 CK @ 800 MHz
    # tBURST is the delay to transfer the Bstof32 =  6 CK @ 800 MHz
    tBURST = '7.5ns'
    # can interleave a Bstof32 from another bank group at tBURST_MIN
    # 16-beats is 8 WCK @2.3 GHz or 2 CK @ 800 MHz
    tBURST_MIN = '2.5ns'
    # tBURST_MAX is the maximum burst delay for same bank group timing
    # this is 8 CK @ 800 MHz
    tBURST_MAX = '10ns'

    # 8 CK @ 800 MHz
    tCCD_L = "10ns"

    # Required RD-to-WR timing is RL+ BL/n + tWCKDQ0/tCK - WL
    # tWCKDQ0/tCK will be 1 CK for most cases
    # For gem5 RL = WL and BL/n is already accounted for with tBURST
    # Result is and additional 1 CK is required
    tRTW = '1.25ns'

    # Default different rank bus delay to 2 CK, @687.5 MHz = 2.5 ns
    tCS = '2.5ns'

    # 2 CK
    tPPD = '2.5ns'

    # 2 command phases can be sent back-to-back or
    # with a gap up to tAAD = 8 CK
    tAAD = '10ns'

# A single LPDDR5 x16 interface (one command/address bus)
# for a single x16 channel with default timings based on initial
# JEDEC specifcation
# 6.4Gbps data rates and 8Gbit die
# Configuring for 16-bank mode with bank-group architecture, burst of 16
class LPDDR5_6400_1x16_BG_BL16(LPDDR5_6400_1x16_BG_BL32):

    # LPDDR5 is a BL16 or BL32 device
    # With BG mode, BL16 and BL32 are supported
    # Use BL16 for smaller access granularity
    burst_length = 16

    # For Bstof16 with BG arch, 2 CK @ 800 MHz with 4:1 clock ratio
    tBURST = '2.5ns'
    tBURST_MIN = '2.5ns'
    # For Bstof16 with BG arch, 4 CK @ 800 MHz with 4:1 clock ratio
    tBURST_MAX = '5ns'

    # 4 CK @ 800 MHz
    tCCD_L = "5ns"


# A single LPDDR5 x16 interface (one command/address bus)
# for a single x16 channel with default timings based on
# initial JEDEC specification
# 6.4Gbps data rates and 8Gbit die
# Configuring for 8-bank mode, burst of 32
class LPDDR5_6400_1x16_8B_BL32(LPDDR5_6400_1x16_BG_BL32):

    # 4kB page with 8B mode
    device_rowbuffer_size = '4kB'

    # LPDDR5 supports configurable bank options
    # 8B  : BL32, all frequencies
    # 16B : BL32 or BL16, <=3.2Gbps
    # 16B with Bank Group Arch (4B/BG): BL32 or BL16, >3.2Gbps
    # Select 8B
    banks_per_rank = 8
    bank_groups_per_rank = 0

    # For Bstof32 with 8B mode, 4 CK @ 800 MHz with 4:1 clock ratio
    tBURST = '5ns'
    tBURST_MIN = '5ns'
    tBURST_MAX = '5ns'

    # Greater of 4 CK or 12 ns
    tWTR = '12ns'

    # Greater of 2 CK or 10 ns
    tRRD = '10ns'

    # With 8B mode tFAW is 40 ns
    tXAW = '40ns'
    activation_limit = 4

    # Reset BG arch timing for 8B mode
    tCCD_L = "0ns"
    tRRD_L = "0ns"
    tWTR_L = "0ns"
