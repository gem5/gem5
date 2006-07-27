from m5 import build_env
from m5.config import *
from Device import DmaDevice
from Pci import PciDevice, PciConfigData

class EtherInt(SimObject):
    type = 'EtherInt'
    abstract = True
    peer = Param.EtherInt(NULL, "peer interface")

class EtherLink(SimObject):
    type = 'EtherLink'
    int1 = Param.EtherInt("interface 1")
    int2 = Param.EtherInt("interface 2")
    delay = Param.Latency('0us', "packet transmit delay")
    delay_var = Param.Latency('0ns', "packet transmit delay variability")
    speed = Param.NetworkBandwidth('1Gbps', "link speed")
    dump = Param.EtherDump(NULL, "dump object")

class EtherBus(SimObject):
    type = 'EtherBus'
    loopback = Param.Bool(True, "send packet back to the sending interface")
    dump = Param.EtherDump(NULL, "dump object")
    speed = Param.NetworkBandwidth('100Mbps', "bus speed in bits per second")

class EtherTap(EtherInt):
    type = 'EtherTap'
    bufsz = Param.Int(10000, "tap buffer size")
    dump = Param.EtherDump(NULL, "dump object")
    port = Param.UInt16(3500, "tap port")

class EtherDump(SimObject):
    type = 'EtherDump'
    file = Param.String("dump file")
    maxlen = Param.Int(96, "max portion of packet data to dump")

if build_env['ALPHA_TLASER']:

    class EtherDev(DmaDevice):
        type = 'EtherDev'
        hardware_address = Param.EthernetAddr(NextEthernetAddr,
            "Ethernet Hardware Address")

        dma_data_free = Param.Bool(False, "DMA of Data is free")
        dma_desc_free = Param.Bool(False, "DMA of Descriptors is free")
        dma_read_delay = Param.Latency('0us', "fixed delay for dma reads")
        dma_read_factor = Param.Latency('0us', "multiplier for dma reads")
        dma_write_delay = Param.Latency('0us', "fixed delay for dma writes")
        dma_write_factor = Param.Latency('0us', "multiplier for dma writes")
        dma_no_allocate = Param.Bool(True, "Should we allocate cache on read")

        rx_filter = Param.Bool(True, "Enable Receive Filter")
        rx_delay = Param.Latency('1us', "Receive Delay")
        tx_delay = Param.Latency('1us', "Transmit Delay")

        intr_delay = Param.Latency('0us', "Interrupt Delay")
        payload_bus = Param.Bus(NULL, "The IO Bus to attach to for payload")
        physmem = Param.PhysicalMemory(Parent.any, "Physical Memory")
        tlaser = Param.Turbolaser(Parent.any, "Turbolaser")

    class EtherDevInt(EtherInt):
        type = 'EtherDevInt'
        device = Param.EtherDev("Ethernet device of this interface")

class EtherDevBase(PciDevice):
    hardware_address = Param.EthernetAddr(NextEthernetAddr,
        "Ethernet Hardware Address")

    clock = Param.Clock('0ns', "State machine processor frequency")

    config_latency = Param.Latency('20ns', "Config read or write latency")

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

class NSGigEPciData(PciConfigData):
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

class NSGigE(EtherDevBase):
    type = 'NSGigE'

    dma_data_free = Param.Bool(False, "DMA of Data is free")
    dma_desc_free = Param.Bool(False, "DMA of Descriptors is free")
    dma_no_allocate = Param.Bool(True, "Should we allocate cache on read")

    configdata = NSGigEPciData()


class NSGigEInt(EtherInt):
    type = 'NSGigEInt'
    device = Param.NSGigE("Ethernet device of this interface")

class SinicPciData(PciConfigData):
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

class Sinic(EtherDevBase):
    type = 'Sinic'

    rx_max_copy = Param.MemorySize('1514B', "rx max copy")
    tx_max_copy = Param.MemorySize('16kB', "tx max copy")
    rx_max_intr = Param.UInt32(10, "max rx packets per interrupt")
    rx_fifo_threshold = Param.MemorySize('384kB', "rx fifo high threshold")
    rx_fifo_low_mark = Param.MemorySize('128kB', "rx fifo low threshold")
    tx_fifo_high_mark = Param.MemorySize('384kB', "tx fifo high threshold")
    tx_fifo_threshold = Param.MemorySize('128kB', "tx fifo low threshold")
    virtual_count = Param.UInt32(1, "Virtualized SINIC")
    zero_copy = Param.Bool(False, "Zero copy receive")
    delay_copy = Param.Bool(False, "Delayed copy transmit")
    virtual_addr = Param.Bool(False, "Virtual addressing")

    configdata = SinicPciData()

class SinicInt(EtherInt):
    type = 'SinicInt'
    device = Param.Sinic("Ethernet device of this interface")
