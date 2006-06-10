from m5.config import *
from MemObject import MemObject

class PioDevice(MemObject):
    type = 'PioDevice'
    abstract = True
    platform = Param.Platform(Parent.any, "Platform this device is part of")
    system = Param.System(Parent.any, "System this device is part of")

class BasicPioDevice(PioDevice):
    type = 'BasicPioDevice'
    abstract = True
    pio_addr = Param.Addr("Device Address")
    pio_latency = Param.Tick(1, "Programmed IO latency in simticks")

class DmaDevice(PioDevice):
    type = 'DmaDevice'
    abstract = True
