from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5 import build_env
from PhysicalMemory import *

class MemoryMode(Enum): vals = ['invalid', 'atomic', 'timing']

class System(SimObject):
    type = 'System'
    physmem = Param.PhysicalMemory(Parent.any, "phsyical memory")
    mem_mode = Param.MemoryMode('atomic', "The mode the memory system is in")
    if build_env['FULL_SYSTEM']:
        boot_cpu_frequency = Param.Frequency(Self.cpu[0].clock.frequency,
                                             "boot processor frequency")
        init_param = Param.UInt64(0, "numerical value to pass into simulator")
        boot_osflags = Param.String("a", "boot flags to pass to the kernel")
        kernel = Param.String("file that contains the kernel code")
        readfile = Param.String("", "file to read startup script from")
        symbolfile = Param.String("", "file to get the symbols from")

class AlphaSystem(System):
    type = 'AlphaSystem'
    console = Param.String("file that contains the console code")
    pal = Param.String("file that contains palcode")
    system_type = Param.UInt64("Type of system we are emulating")
    system_rev = Param.UInt64("Revision of system we are emulating")

class SparcSystem(System):
    type = 'SparcSystem'
    _rom_base = 0xfff0000000
    # ROM for OBP/Reset/Hypervisor
    rom = Param.PhysicalMemory(PhysicalMemory(range = AddrRange(_rom_base, size = '8MB')),
            "Memory to hold the ROM data")

    reset_addr = Param.Addr(_rom_base, "Address to load ROM at")
    hypervisor_addr = Param.Addr(Addr('64kB') + _rom_base,
                                 "Address to load hypervisor at")
    openboot_addr = Param.Addr(Addr('512kB') + _rom_base,
                               "Address to load openboot at")

    reset_bin = Param.String("file that contains the reset code")
    hypervisor_bin = Param.String("file that contains the hypervisor code")
    openboot_bin = Param.String("file that contains the openboot code")

