# Copyright (c) 2015 ARM Limited
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
# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert

from m5.defines import buildEnv
from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5.objects.PciDevice import PciDevice

ETHERNET_ROLE = 'ETHERNET'
Port.compat(ETHERNET_ROLE, ETHERNET_ROLE)

class EtherInt(Port):
    def __init__(self, desc):
        super(EtherInt, self).__init__(ETHERNET_ROLE, desc)

class VectorEtherInt(VectorPort):
    def __init__(self, desc):
        super(VectorEtherInt, self).__init__(ETHERNET_ROLE, desc)

class EtherLink(SimObject):
    type = 'EtherLink'
    cxx_header = "dev/net/etherlink.hh"
    int0 = EtherInt("interface 0")
    int1 = EtherInt("interface 1")
    delay = Param.Latency('0us', "packet transmit delay")
    delay_var = Param.Latency('0ns', "packet transmit delay variability")
    speed = Param.NetworkBandwidth('1Gbps', "link speed")
    dump = Param.EtherDump(NULL, "dump object")

class DistEtherLink(SimObject):
    type = 'DistEtherLink'
    cxx_header = "dev/net/dist_etherlink.hh"
    int0 = EtherInt("interface 0")
    delay = Param.Latency('0us', "packet transmit delay")
    delay_var = Param.Latency('0ns', "packet transmit delay variability")
    speed = Param.NetworkBandwidth('1Gbps', "link speed")
    dump = Param.EtherDump(NULL, "dump object")
    dist_rank = Param.UInt32('0', "Rank of this gem5 process (dist run)")
    dist_size = Param.UInt32('1', "Number of gem5 processes (dist run)")
    sync_start = Param.Latency('5200000000000t', "first dist sync barrier")
    sync_repeat = Param.Latency('10us', "dist sync barrier repeat")
    server_name = Param.String('localhost', "Message server name")
    server_port = Param.UInt32('2200', "Message server port")
    is_switch = Param.Bool(False, "true if this a link in etherswitch")
    dist_sync_on_pseudo_op = Param.Bool(False, "Start sync with pseudo_op")
    num_nodes = Param.UInt32('2', "Number of simulate nodes")

class EtherBus(SimObject):
    type = 'EtherBus'
    cxx_header = "dev/net/etherbus.hh"
    loopback = Param.Bool(True, "send packet back to the sending interface")
    dump = Param.EtherDump(NULL, "dump object")
    speed = Param.NetworkBandwidth('100Mbps', "bus speed in bits per second")

class EtherSwitch(SimObject):
    type = 'EtherSwitch'
    cxx_header = "dev/net/etherswitch.hh"
    dump = Param.EtherDump(NULL, "dump object")
    fabric_speed = Param.NetworkBandwidth('10Gbps', "switch fabric speed in bits "
                                          "per second")
    interface = VectorEtherInt("Ethernet Interface")
    output_buffer_size = Param.MemorySize('1MB', "size of output port buffers")
    delay = Param.Latency('0us', "packet transmit delay")
    delay_var = Param.Latency('0ns', "packet transmit delay variability")
    time_to_live = Param.Latency('10ms', "time to live of MAC address maping")

class EtherTapBase(SimObject):
    type = 'EtherTapBase'
    abstract = True
    cxx_header = "dev/net/ethertap.hh"
    bufsz = Param.Int(10000, "tap buffer size")
    dump = Param.EtherDump(NULL, "dump object")
    tap = EtherInt("Ethernet interface to connect to gem5's network")

if buildEnv['USE_TUNTAP']:
    class EtherTap(EtherTapBase):
        type = 'EtherTap'
        cxx_header = "dev/net/ethertap.hh"
        tun_clone_device = Param.String('/dev/net/tun',
                                        "Path to the tun clone device node")
        tap_device_name = Param.String('gem5-tap', "Tap device name")

class EtherTapStub(EtherTapBase):
    type = 'EtherTapStub'
    cxx_header = "dev/net/ethertap.hh"
    port = Param.UInt16(3500, "Port helper should send packets to")

class EtherDump(SimObject):
    type = 'EtherDump'
    cxx_header = "dev/net/etherdump.hh"
    file = Param.String("dump file")
    maxlen = Param.Int(96, "max portion of packet data to dump")

class EtherDevice(PciDevice):
    type = 'EtherDevice'
    abstract = True
    cxx_header = "dev/net/etherdevice.hh"
    interface = EtherInt("Ethernet Interface")

class IGbE(EtherDevice):
    # Base class for two IGbE adapters listed above
    type = 'IGbE'
    cxx_header = "dev/net/i8254xGBe.hh"
    hardware_address = Param.EthernetAddr(NextEthernetAddr,
        "Ethernet Hardware Address")
    rx_fifo_size = Param.MemorySize('384kB', "Size of the rx FIFO")
    tx_fifo_size = Param.MemorySize('384kB', "Size of the tx FIFO")
    rx_desc_cache_size = Param.Int(64,
        "Number of enteries in the rx descriptor cache")
    tx_desc_cache_size = Param.Int(64,
        "Number of enteries in the rx descriptor cache")
    VendorID = 0x8086
    SubsystemID = 0x1008
    SubsystemVendorID = 0x8086
    Status = 0x0000
    SubClassCode = 0x00
    ClassCode = 0x02
    ProgIF = 0x00
    BAR0 = 0x00000000
    BAR1 = 0x00000000
    BAR2 = 0x00000000
    BAR3 = 0x00000000
    BAR4 = 0x00000000
    BAR5 = 0x00000000
    MaximumLatency = 0x00
    MinimumGrant = 0xff
    InterruptLine = 0x1e
    InterruptPin = 0x01
    BAR0Size = '128kB'
    wb_delay = Param.Latency('10ns', "delay before desc writeback occurs")
    fetch_delay = Param.Latency('10ns', "delay before desc fetch occurs")
    fetch_comp_delay = Param.Latency('10ns', "delay after desc fetch occurs")
    wb_comp_delay = Param.Latency('10ns', "delay after desc wb occurs")
    tx_read_delay = Param.Latency('0ns', "delay after tx dma read")
    rx_write_delay = Param.Latency('0ns', "delay after rx dma read")
    phy_pid = Param.UInt16("Phy PID that corresponds to device ID")
    phy_epid = Param.UInt16("Phy EPID that corresponds to device ID")

class IGbE_e1000(IGbE):
    # Older Intel 8254x based gigabit ethernet adapter
    # Uses Intel e1000 driver
    DeviceID = 0x1075
    phy_pid = 0x02A8
    phy_epid = 0x0380

class IGbE_igb(IGbE):
    # Newer Intel 8257x based gigabit ethernet adapter
    # Uses Intel igb driver and in theory supports packet splitting and LRO
    DeviceID = 0x10C9
    phy_pid = 0x0141
    phy_epid = 0x0CC0

class EtherDevBase(EtherDevice):
    type = 'EtherDevBase'
    abstract = True
    cxx_header = "dev/net/etherdevice.hh"

    hardware_address = Param.EthernetAddr(NextEthernetAddr,
        "Ethernet Hardware Address")

    dma_read_delay = Param.Latency('0us', "fixed delay for dma reads")
    dma_read_factor = Param.Latency('0us', "multiplier for dma reads")
    dma_write_delay = Param.Latency('0us', "fixed delay for dma writes")
    dma_write_factor = Param.Latency('0us', "multiplier for dma writes")

    rx_delay = Param.Latency('1us', "Receive Delay")
    tx_delay = Param.Latency('1us', "Transmit Delay")
    rx_fifo_size = Param.MemorySize('512kB', "max size of rx fifo")
    tx_fifo_size = Param.MemorySize('512kB', "max size of tx fifo")

    rx_filter = Param.Bool(True, "Enable Receive Filter")
    intr_delay = Param.Latency('10us', "Interrupt propagation delay")
    rx_thread = Param.Bool(False, "dedicated kernel thread for transmit")
    tx_thread = Param.Bool(False, "dedicated kernel threads for receive")
    rss = Param.Bool(False, "Receive Side Scaling")

class NSGigE(EtherDevBase):
    type = 'NSGigE'
    cxx_header = "dev/net/ns_gige.hh"

    dma_data_free = Param.Bool(False, "DMA of Data is free")
    dma_desc_free = Param.Bool(False, "DMA of Descriptors is free")
    dma_no_allocate = Param.Bool(True, "Should we allocate cache on read")

    VendorID = 0x100B
    DeviceID = 0x0022
    Status = 0x0290
    SubClassCode = 0x00
    ClassCode = 0x02
    ProgIF = 0x00
    BAR0 = 0x00000001
    BAR1 = 0x00000000
    BAR2 = 0x00000000
    BAR3 = 0x00000000
    BAR4 = 0x00000000
    BAR5 = 0x00000000
    MaximumLatency = 0x34
    MinimumGrant = 0xb0
    InterruptLine = 0x1e
    InterruptPin = 0x01
    BAR0Size = '256B'
    BAR1Size = '4kB'



class Sinic(EtherDevBase):
    type = 'Sinic'
    cxx_class = 'Sinic::Device'
    cxx_header = "dev/net/sinic.hh"

    rx_max_copy = Param.MemorySize('1514B', "rx max copy")
    tx_max_copy = Param.MemorySize('16kB', "tx max copy")
    rx_max_intr = Param.UInt32(10, "max rx packets per interrupt")
    rx_fifo_threshold = Param.MemorySize('384kB', "rx fifo high threshold")
    rx_fifo_low_mark = Param.MemorySize('128kB', "rx fifo low threshold")
    tx_fifo_high_mark = Param.MemorySize('384kB', "tx fifo high threshold")
    tx_fifo_threshold = Param.MemorySize('128kB', "tx fifo low threshold")
    virtual_count = Param.UInt32(1, "Virtualized SINIC")
    zero_copy_size = Param.UInt32(64, "Bytes to copy if below threshold")
    zero_copy_threshold = Param.UInt32(256,
        "Only zero copy above this threshold")
    zero_copy = Param.Bool(False, "Zero copy receive")
    delay_copy = Param.Bool(False, "Delayed copy transmit")
    virtual_addr = Param.Bool(False, "Virtual addressing")

    VendorID = 0x1291
    DeviceID = 0x1293
    Status = 0x0290
    SubClassCode = 0x00
    ClassCode = 0x02
    ProgIF = 0x00
    BAR0 = 0x00000000
    BAR1 = 0x00000000
    BAR2 = 0x00000000
    BAR3 = 0x00000000
    BAR4 = 0x00000000
    BAR5 = 0x00000000
    MaximumLatency = 0x34
    MinimumGrant = 0xb0
    InterruptLine = 0x1e
    InterruptPin = 0x01
    BAR0Size = '64kB'


