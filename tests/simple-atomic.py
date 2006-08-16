import m5
from m5.objects import *
m5.AddToPath('../configs/common')
from SEConfig import *

system = System(cpu = AtomicSimpleCPU(),
                physmem = PhysicalMemory(),
                membus = Bus())
system.physmem.port = system.membus.port
system.cpu.connectMemPorts(system.membus)

root = Root(system = system)
