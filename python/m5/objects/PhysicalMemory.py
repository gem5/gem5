from m5 import *
from MemObject import *

class PhysicalMemory(MemObject):
    type = 'PhysicalMemory'
    range = Param.AddrRange("Device Address")
    file = Param.String('', "memory mapped file")
    latency = Param.Latency('10ns', "latency of an access")
