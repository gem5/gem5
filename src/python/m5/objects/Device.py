from m5.config import *
from MemObject import MemObject

class PioDevice(MemObject):
    type = 'PioDevice'
    abstract = True
    pio = Port("Programmed I/O port")
    platform = Param.Platform(Parent.any, "Platform this device is part of")
    system = Param.System(Parent.any, "System this device is part of")

class BasicPioDevice(PioDevice):
    type = 'BasicPioDevice'
    abstract = True
    pio_addr = Param.Addr("Device Address")
    pio_latency = Param.Latency('1ns', "Programmed IO latency in simticks")

class DmaDevice(PioDevice):
    type = 'DmaDevice'
    abstract = True
    dma = Port("DMA port")
