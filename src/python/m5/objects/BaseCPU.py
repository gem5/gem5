from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5 import build_env
from AlphaTLB import AlphaDTB, AlphaITB
from SparcTLB import SparcDTB, SparcITB
from Bus import Bus
import sys

class BaseCPU(SimObject):
    type = 'BaseCPU'
    abstract = True

    system = Param.System(Parent.any, "system object")
    cpu_id = Param.Int("CPU identifier")

    if build_env['FULL_SYSTEM']:
        do_quiesce = Param.Bool(True, "enable quiesce instructions")
        do_checkpoint_insts = Param.Bool(True,
            "enable checkpoint pseudo instructions")
        do_statistics_insts = Param.Bool(True,
            "enable statistics pseudo instructions")

        if build_env['TARGET_ISA'] == 'sparc':
            dtb = Param.SparcDTB(SparcDTB(), "Data TLB")
            itb = Param.SparcITB(SparcITB(), "Instruction TLB")
        elif build_env['TARGET_ISA'] == 'alpha':
            dtb = Param.AlphaDTB(AlphaDTB(), "Data TLB")
            itb = Param.AlphaITB(AlphaITB(), "Instruction TLB")
        else:
            print "Unknown architecture, can't pick TLBs"
            sys.exit(1)
    else:
        workload = VectorParam.Process("processes to run")

    max_insts_all_threads = Param.Counter(0,
        "terminate when all threads have reached this inst count")
    max_insts_any_thread = Param.Counter(0,
        "terminate when any thread reaches this inst count")
    max_loads_all_threads = Param.Counter(0,
        "terminate when all threads have reached this load count")
    max_loads_any_thread = Param.Counter(0,
        "terminate when any thread reaches this load count")
    progress_interval = Param.Tick(0, "interval to print out the progress message")

    defer_registration = Param.Bool(False,
        "defer registration with system (for sampling)")

    clock = Param.Clock(Parent.clock, "clock speed")

    _mem_ports = []

    def connectMemPorts(self, bus):
        for p in self._mem_ports:
            exec('self.%s = bus.port' % p)

    def addPrivateSplitL1Caches(self, ic, dc):
        assert(len(self._mem_ports) == 2)
        self.icache = ic
        self.dcache = dc
        self.icache_port = ic.cpu_side
        self.dcache_port = dc.cpu_side
        self._mem_ports = ['icache.mem_side', 'dcache.mem_side']

    def addTwoLevelCacheHierarchy(self, ic, dc, l2c):
        self.addPrivateSplitL1Caches(ic, dc)
        self.toL2Bus = Bus()
        self.connectMemPorts(self.toL2Bus)
        self.l2cache = l2c
        self.l2cache.cpu_side = self.toL2Bus.port
        self._mem_ports = ['l2cache.mem_side']
