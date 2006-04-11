from m5 import *
from Memory import Memory

class PhysicalMemory(Memory):
    type = 'PhysicalMemory'
    range = Param.AddrRange("Device Address")
    file = Param.String('', "memory mapped file")
