from m5 import *
from Device import FooPioDevice
from Platform import Platform

class Tsunami(Platform):
    type = 'Tsunami'
    pciconfig = Param.PciConfigAll("PCI configuration")
    system = Param.System(Parent.any, "system")

class TsunamiCChip(FooPioDevice):
    type = 'TsunamiCChip'
    tsunami = Param.Tsunami(Parent.any, "Tsunami")

class TsunamiFake(FooPioDevice):
    type = 'TsunamiFake'
    size = Param.Addr("Size of address range")

class TsunamiIO(FooPioDevice):
    type = 'TsunamiIO'
    time = Param.UInt64(1136073600,
        "System time to use (0 for actual time, default is 1/1/06)")
    tsunami = Param.Tsunami(Parent.any, "Tsunami")
    frequency = Param.Frequency('1024Hz', "frequency of interrupts")

class TsunamiPChip(FooPioDevice):
    type = 'TsunamiPChip'
    tsunami = Param.Tsunami(Parent.any, "Tsunami")
