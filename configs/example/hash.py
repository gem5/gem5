from __future__ import print_function
from __future__ import absolute_import
import sys
import argparse
import m5
from m5.objects import *
from m5.util import *
addToPath('../')
from common import MemConfig
from common import HMC

##cache configuartion
class L1Cache(Cache):
    """Simple L1 Cache with default values"""

    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    def __init__(self, options=None):
        super(L1Cache, self).__init__()
        pass

    def connectBus(self, bus):
        """Connect this cache to a memory-side bus"""
        self.mem_side = bus.slave

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU-side port
           This must be defined in a subclass"""
        raise NotImplementedError

class L1ICache(L1Cache):
    """Simple L1 instruction cache with default values"""

    # Set the default size
    size = '16kB'

    def __init__(self, opts=None):
        super(L1ICache, self).__init__(opts)
        self.size = '16kB'

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU icache port"""
        self.cpu_side = cpu.icache_port

class L1DCache(L1Cache):
    """Simple L1 data cache with default values"""

    # Set the default size

    def __init__(self, opts=None):
        super(L1DCache, self).__init__(opts)
        self.size = '64kB'

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU dcache port"""
        self.cpu_side = cpu.dcache_port

class L2Cache(Cache):
    """Simple L2 Cache with default values"""

    # Default parameters
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    def __init__(self, opts=None):
        super(L2Cache, self).__init__()
        self.size = '256kB'

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.master

    def connectMemSideBus(self, bus):
        self.mem_side = bus.slave



#adding arguments
parser = argparse.ArgumentParser()
HMC.add_options(parser)
options = parser.parse_args()

#creating sysytem and configuring the basic stuff
system = System()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange('4GB')]
clk = '1GHz'
vd = VoltageDomain(voltage='1V')
system.clk_domain = SrcClockDomain(clock=clk, voltage_domain=vd)
system.cpu =TimingSimpleCPU()
system.membus = SystemXBar()
MemConfig.config_mem(options, system)

#adding ports and cache 
system.cpu.icache = L1ICache(options)
system.cpu.dcache = L1DCache(options)
system.cpu.icache.connectCPU(system.cpu)
system.cpu.dcache.connectCPU(system.cpu)
system.l2bus = L2XBar()
system.cpu.icache.connectBus(system.l2bus)
system.cpu.dcache.connectBus(system.l2bus)
system.l2cache = L2Cache(options)
system.l2cache.connectCPUSideBus(system.l2bus)
system.l2cache.connectMemSideBus(system.membus)
system.cpu.createInterruptController()
system.cpu.interrupts[0].pio = system.membus.master
system.cpu.interrupts[0].int_master = system.membus.slave
system.cpu.interrupts[0].int_slave = system.membus.master
system.system_port = system.membus.slave


#adding binary and simulating
binary = options.cmd
process = Process()
process.cmd = [binary]
system.cpu.workload = process
system.cpu.createThreads()
root = Root(full_system=False, system=system)
m5.instantiate()
print("Beginning simulation!")
exit_event=m5.simulate()
print('Exiting @ tick {} because {}'
      .format(m5.curTick(), exit_event.getCause()))
