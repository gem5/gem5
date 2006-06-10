from m5.config import *
from Device import BasicPioDevice
from Platform import Platform

class Tsunami(Platform):
    type = 'Tsunami'
#    pciconfig = Param.PciConfigAll("PCI configuration")
    system = Param.System(Parent.any, "system")

class TsunamiCChip(BasicPioDevice):
    type = 'TsunamiCChip'
    tsunami = Param.Tsunami(Parent.any, "Tsunami")

class IsaFake(BasicPioDevice):
    type = 'IsaFake'
    pio_size = Param.Addr(0x8, "Size of address range")

class TsunamiIO(BasicPioDevice):
    type = 'TsunamiIO'
    time = Param.UInt64(1136073600,
        "System time to use (0 for actual time, default is 1/1/06)")
    tsunami = Param.Tsunami(Parent.any, "Tsunami")
    frequency = Param.Frequency('1024Hz', "frequency of interrupts")

class TsunamiPChip(BasicPioDevice):
    type = 'TsunamiPChip'
    tsunami = Param.Tsunami(Parent.any, "Tsunami")
