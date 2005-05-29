from m5 import *
from FunctionalMemory import FunctionalMemory

class PhysicalMemory(FunctionalMemory):
    type = 'PhysicalMemory'
    range = Param.AddrRange("Device Address")
    file = Param.String('', "memory mapped file")
    mmu = Param.MemoryController(Parent.any, "Memory Controller")
