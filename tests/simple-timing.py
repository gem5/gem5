import m5
from m5.objects import *
m5.AddToPath('../configs/common')
from SEConfig import *

class MyCache(BaseCache):
    assoc = 2
    block_size = 64
    latency = 1
    mshrs = 10
    tgts_per_mshr = 5

cpu = TimingSimpleCPU()
cpu.addTwoLevelCacheHierarchy(MyCache(size = '128kB'), MyCache(size = '256kB'),
                              MyCache(size = '2MB'))

system = System(cpu = cpu,
                physmem = PhysicalMemory(),
                membus = Bus())
system.physmem.port = system.membus.port
cpu.connectMemPorts(system.membus)

root = Root(system = system)
