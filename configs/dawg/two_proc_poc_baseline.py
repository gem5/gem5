# insecure baseline configuration

# system specification as per Table I simulated system specifications:
# 32KB, 8-way L1s
# 256KB, 8-way private L2 per core
# 16MB, 16-way shared L3


import m5
from m5.objects import *


class L3Cache(Cache):
    assoc = 16
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 64
    tgts_per_mshr = 16
    size = "16MB"

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


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
    size = "256kB"
    assoc = 8
    tag_latency = 6
    data_latency = 6
    response_latency = 6
    mshrs = 8
    tgts_per_mshr = 16

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


system = System()
system.clk_domain = SrcClockDomain(
    clock="3GHz", voltage_domain=VoltageDomain()
)
system.mem_mode = "timing"
system.mem_ranges = [AddrRange("8192MB")]
system.cache_line_size = 64

num_cores = 8
system.cpu = [DerivO3CPU(cpu_id=i) for i in range(num_cores)]

for cpu in system.cpu:
    cpu.icache = L1ICache()
    cpu.dcache = L1DCache()

    cpu.l2bus = L2XBar()
    cpu.l2cache = L2Cache()

system.l3cache = L3Cache()

system.membus = SystemXBar()
system.l3bus = SystemXBar()

for i, cpu in enumerate(system.cpu):
    cpu.icache.connectCPU(cpu)
    cpu.dcache.connectCPU(cpu)

    cpu.icache.connectBus(cpu.l2bus)
    cpu.dcache.connectBus(cpu.l2bus)

    cpu.l2cache.connectCPUSideBus(cpu.l2bus)
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

# binary = "tests/test-progs/threads/bin/x86/linux/threads"
# binary = "tests/test-progs/hello/bin/x86/linux/hello"

binary = "/home/hoop3r/School/Architecture/gem5-dawg-modification/external-microbench/ML2_BW_ldst/bench.X86"

system.workload = SEWorkload.init_compatible(binary)

for i, cpu in enumerate(system.cpu):
    process = Process()
    process.cmd = [binary]
    process.pid = 300 + i
    cpu.workload = process
    cpu.createThreads()

m5.instantiate()

exit_event = m5.simulate()
print(
    f"BASELINE Exiting @ tick {m5.curTick()} because {exit_event.getCause()}"
)
