from m5 import *

class HelloWorld(AlphaLiveProcess):
    executable = '../configs/test/hello'
    cmd = 'hello'

magicbus = Bus()
mem = PhysicalMemory(bus=magicbus)
cpu = SimpleCPU(workload=HelloWorld(), mem=magicbus)
system = System(physmem=mem, cpu=cpu)
root = Root(system=system)
