from m5.config import *
from Device import BasicPioDevice, DmaDevice, PioDevice

class PciConfigData(SimObject):
    type = 'PciConfigData'
    VendorID = Param.UInt16("Vendor ID")
    DeviceID = Param.UInt16("Device ID")
    Command = Param.UInt16(0, "Command")
    Status = Param.UInt16(0, "Status")
    Revision = Param.UInt8(0, "Device")
    ProgIF = Param.UInt8(0, "Programming Interface")
    SubClassCode = Param.UInt8(0, "Sub-Class Code")
    ClassCode = Param.UInt8(0, "Class Code")
    CacheLineSize = Param.UInt8(0, "System Cacheline Size")
    LatencyTimer = Param.UInt8(0, "PCI Latency Timer")
    HeaderType = Param.UInt8(0, "PCI Header Type")
    BIST = Param.UInt8(0, "Built In Self Test")

    BAR0 = Param.UInt32(0x00, "Base Address Register 0")
    BAR1 = Param.UInt32(0x00, "Base Address Register 1")
    BAR2 = Param.UInt32(0x00, "Base Address Register 2")
    BAR3 = Param.UInt32(0x00, "Base Address Register 3")
    BAR4 = Param.UInt32(0x00, "Base Address Register 4")
    BAR5 = Param.UInt32(0x00, "Base Address Register 5")
    BAR0Size = Param.MemorySize32('0B', "Base Address Register 0 Size")
    BAR1Size = Param.MemorySize32('0B', "Base Address Register 1 Size")
    BAR2Size = Param.MemorySize32('0B', "Base Address Register 2 Size")
    BAR3Size = Param.MemorySize32('0B', "Base Address Register 3 Size")
    BAR4Size = Param.MemorySize32('0B', "Base Address Register 4 Size")
    BAR5Size = Param.MemorySize32('0B', "Base Address Register 5 Size")

    CardbusCIS = Param.UInt32(0x00, "Cardbus Card Information Structure")
    SubsystemID = Param.UInt16(0x00, "Subsystem ID")
    SubsystemVendorID = Param.UInt16(0x00, "Subsystem Vendor ID")
    ExpansionROM = Param.UInt32(0x00, "Expansion ROM Base Address")
    InterruptLine = Param.UInt8(0x00, "Interrupt Line")
    InterruptPin = Param.UInt8(0x00, "Interrupt Pin")
    MaximumLatency = Param.UInt8(0x00, "Maximum Latency")
    MinimumGrant = Param.UInt8(0x00, "Minimum Grant")

class PciConfigAll(PioDevice):
    type = 'PciConfigAll'
    pio_latency = Param.Tick(1, "Programmed IO latency in simticks")
    bus = Param.UInt8(0x00, "PCI bus to act as config space for")
    size = Param.MemorySize32('16MB', "Size of config space")


class PciDevice(DmaDevice):
    type = 'PciDevice'
    abstract = True
    config = Port("PCI configuration space port")
    pci_bus = Param.Int("PCI bus")
    pci_dev = Param.Int("PCI device number")
    pci_func = Param.Int("PCI function code")
    pio_latency = Param.Tick(1, "Programmed IO latency in simticks")
    configdata = Param.PciConfigData(Parent.any, "PCI Config data")

class PciFake(PciDevice):
    type = 'PciFake'
