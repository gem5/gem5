from m5 import *

class HelloWorld(LiveProcess):
    executable = '../configs/test/hello'
    cmd = 'hello'

mem = PhysicalMemory()
cpu = SimpleCPU(workload=HelloWorld(), mem=mem)
system = System(physmem=mem, cpu=cpu)
root = Root(system=system)
