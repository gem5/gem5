from m5 import *
from Device import DmaDevice
from Pci import PciDevice

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

    physmem = Param.PhysicalMemory(Parent.any, "Physical Memory")

    hier = Param.HierParams(Parent.any, "Hierarchy global variables")
    payload_bus = Param.Bus(NULL, "The IO Bus to attach to for payload")
    dma_read_delay = Param.Latency('0us', "fixed delay for dma reads")
    dma_read_factor = Param.Latency('0us', "multiplier for dma reads")
    dma_write_delay = Param.Latency('0us', "fixed delay for dma writes")
    dma_write_factor = Param.Latency('0us', "multiplier for dma writes")
    dma_no_allocate = Param.Bool(True, "Should we allocate cache on read")

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

    dma_data_free = Param.Bool(False, "DMA of Data is free")
    dma_desc_free = Param.Bool(False, "DMA of Descriptors is free")


class NSGigEInt(EtherInt):
    type = 'NSGigEInt'
    device = Param.NSGigE("Ethernet device of this interface")

class Sinic(EtherDevBase):
    type = 'Sinic'

    rx_max_copy = Param.MemorySize('1514B', "rx max copy")
    tx_max_copy = Param.MemorySize('16kB', "tx max copy")
    rx_max_intr = Param.UInt32(10, "max rx packets per interrupt")
    rx_fifo_threshold = Param.MemorySize('48kB', "rx fifo high threshold")
    tx_fifo_threshold = Param.MemorySize('16kB', "tx fifo low threshold")

class SinicInt(EtherInt):
    type = 'SinicInt'
    device = Param.Sinic("Ethernet device of this interface")
