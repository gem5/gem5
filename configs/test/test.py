from m5 import *

class HelloWorld(AlphaLiveProcess):
    executable = '../configs/test/hello'
    cmd = 'hello'

magicbus = Bus()
mem = PhysicalMemory()
cpu = AtomicSimpleCPU(workload=HelloWorld(), mem=magicbus)
system = System(physmem=mem, cpu=cpu)
system.c1 =  Connector(side_a=mem, side_b=magicbus)
root = Root(system=system)
