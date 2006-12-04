from m5.params import *
from m5.proxy import *
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
    dma = Port(Self.pio.peerObj.port, "DMA port")

class IsaFake(BasicPioDevice):
    type = 'IsaFake'
    pio_size = Param.Addr(0x8, "Size of address range")
    ret_data8 = Param.UInt8(0xFF, "Default data to return")
    ret_data16 = Param.UInt16(0xFFFF, "Default data to return")
    ret_data32 = Param.UInt32(0xFFFFFFFF, "Default data to return")
    ret_data64 = Param.UInt64(0xFFFFFFFFFFFFFFFF, "Default data to return")
    ret_bad_addr = Param.Bool(False, "Return pkt status bad address on access")
    update_data = Param.Bool(False, "Update the data that is returned on writes")
    warn_access = Param.String("", "String to print when device is accessed")

class BadAddr(IsaFake):
    ret_bad_addr = Param.Bool(True, "Return pkt status bad address on access")


