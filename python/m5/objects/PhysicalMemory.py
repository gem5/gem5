from m5 import *
from Memory import Memory

class PhysicalMemory(Memory):
    type = 'PhysicalMemory'
    range = Param.AddrRange("Device Address")
    file = Param.String('', "memory mapped file")
    bus = Param.MemObject("Bus to attach to")
    if build_env['FULL_SYSTEM']:
        mmu = Param.MemoryController(Parent.any, "Memory Controller")
