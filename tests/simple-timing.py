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
cpu.icache = MyCache(size = '128kB')
cpu.dcache = MyCache(size = '256kB')
cpu.l2cache = MyCache(size = '2MB')

cpu.icache_port = cpu.icache.cpu_side
cpu.dcache_port = cpu.dcache.cpu_side

root = makeSESystem(cpu)
