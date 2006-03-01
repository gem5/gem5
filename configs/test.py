from m5 import *
AddToPath('/z/stever/bk/m5-test')
import Benchmarks

mem = PhysicalMemory()
cpu = SimpleCPU(workload=Benchmarks.HelloWorld(), mem=mem)
system = System(physmem=mem, cpu=cpu)
root = Root(system=system)
