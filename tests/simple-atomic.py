import m5
from m5.objects import *
m5.AddToPath('../configs/common')
from SEConfig import *

system = System(cpu = AtomicSimpleCPU(),
                physmem = PhysicalMemory(),
                membus = Bus())
system.physmem.port = system.membus.port
system.cpu.icache_port = system.membus.port
system.cpu.dcache_port = system.membus.port

root = Root(system = system)

