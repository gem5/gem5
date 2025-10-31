import os

import m5
from m5.objects import *


class L1Cache(Cache):
    assoc = 8
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    size = "32kB"

    def connectBus(self, bus):
        self.mem_side = bus.cpu_side_ports


class L1ICache(L1Cache):
    def connectCPU(self, cpu):
        self.cpu_side = cpu.icache_port


class L1DCache(L1Cache):
    def connectCPU(self, cpu):
        self.cpu_side = cpu.dcache_port


class L2Cache(Cache):
    assoc = 8
    tag_latency = 6
    data_latency = 6
    response_latency = 6
    mshrs = 8
    tgts_per_mshr = 12
    size = "256kB"

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


class L3Cache(Cache):
    assoc = 15
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 16
    tgts_per_mshr = 16
    size = "30MB"

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


system = System()
system.clk_domain = SrcClockDomain(
    clock="2.5GHz", voltage_domain=VoltageDomain()
)
system.mem_mode = "timing"

system.mem_ranges = [AddrRange("8192MB")]
# system.mem_ranges = [AddrRange("16GB")]
system.cache_line_size = 64

# num_cores = 12
num_cores = 1
system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(num_cores)]

for cpu in system.cpu:
    cpu.icache = L1ICache()
    cpu.dcache = L1DCache()
    cpu.l2cache = L2Cache()

system.l3cache = L3Cache()

system.membus = SystemXBar()
system.l3bus = SystemXBar()
system.l2bus = [L2XBar() for _ in range(num_cores)]

for i, cpu in enumerate(system.cpu):
    cpu.icache.connectCPU(cpu)
    cpu.dcache.connectCPU(cpu)
    cpu.icache.connectBus(system.l2bus[i])
    cpu.dcache.connectBus(system.l2bus[i])
    cpu.l2cache.connectCPUSideBus(system.l2bus[i])
    cpu.l2cache.connectMemSideBus(system.l3bus)
    cpu.createInterruptController()
    cpu.interrupts[0].pio = system.membus.mem_side_ports
    cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
    cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.l3cache.connectCPUSideBus(system.l3bus)
system.l3cache.connectMemSideBus(system.membus)

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8(range=system.mem_ranges[0])
system.mem_ctrl.port = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports
root = Root(full_system=False, system=system)

# thispath = os.path.dirname(os.path.realpath(__file__))
binary = "tests/test-progs/hello/bin/x86/linux/hello"

system.workload = SEWorkload.init_compatible(binary)

for cpu in system.cpu:
    process = Process()
    process.cmd = [binary]
    cpu.workload = process
    cpu.createThreads()

m5.instantiate()
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
