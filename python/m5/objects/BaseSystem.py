from m5 import *
class BaseSystem(SimObject):
    type = 'BaseSystem'
    abstract = True
    boot_cpu_frequency = Param.Frequency(Self.cpu[0].clock.frequency,
                                         "boot processor frequency")
    memctrl = Param.MemoryController(Parent.any, "memory controller")
    physmem = Param.PhysicalMemory(Parent.any, "phsyical memory")
    kernel = Param.String("file that contains the kernel code")
    console = Param.String("file that contains the console code")
    pal = Param.String("file that contains palcode")
    readfile = Param.String("", "file to read startup script from")
    init_param = Param.UInt64(0, "numerical value to pass into simulator")
    boot_osflags = Param.String("a", "boot flags to pass to the kernel")
    system_type = Param.UInt64("Type of system we are emulating")
    system_rev = Param.UInt64("Revision of system we are emulating")
    bin = Param.Bool(False, "is this system binned")
    binned_fns = VectorParam.String([], "functions broken down and binned")
