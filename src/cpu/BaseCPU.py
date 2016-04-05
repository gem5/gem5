# Copyright (c) 2012-2013, 2015 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2005-2008 The Regents of The University of Michigan
# Copyright (c) 2011 Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Nathan Binkert
#          Rick Strong
#          Andreas Hansson

import sys

from m5.defines import buildEnv
from m5.params import *
from m5.proxy import *

from XBar import L2XBar
from InstTracer import InstTracer
from CPUTracers import ExeTracer
from MemObject import MemObject
from ClockDomain import *

default_tracer = ExeTracer()

if buildEnv['TARGET_ISA'] == 'alpha':
    from AlphaTLB import AlphaDTB, AlphaITB
    from AlphaInterrupts import AlphaInterrupts
    from AlphaISA import AlphaISA
    isa_class = AlphaISA
elif buildEnv['TARGET_ISA'] == 'sparc':
    from SparcTLB import SparcTLB
    from SparcInterrupts import SparcInterrupts
    from SparcISA import SparcISA
    isa_class = SparcISA
elif buildEnv['TARGET_ISA'] == 'x86':
    from X86TLB import X86TLB
    from X86LocalApic import X86LocalApic
    from X86ISA import X86ISA
    isa_class = X86ISA
elif buildEnv['TARGET_ISA'] == 'mips':
    from MipsTLB import MipsTLB
    from MipsInterrupts import MipsInterrupts
    from MipsISA import MipsISA
    isa_class = MipsISA
elif buildEnv['TARGET_ISA'] == 'arm':
    from ArmTLB import ArmTLB, ArmStage2IMMU, ArmStage2DMMU
    from ArmInterrupts import ArmInterrupts
    from ArmISA import ArmISA
    isa_class = ArmISA
elif buildEnv['TARGET_ISA'] == 'power':
    from PowerTLB import PowerTLB
    from PowerInterrupts import PowerInterrupts
    from PowerISA import PowerISA
    isa_class = PowerISA

class BaseCPU(MemObject):
    type = 'BaseCPU'
    abstract = True
    cxx_header = "cpu/base.hh"

    @classmethod
    def export_methods(cls, code):
        code('''
    void switchOut();
    void takeOverFrom(BaseCPU *cpu);
    bool switchedOut();
    void flushTLBs();
    Counter totalInsts();
    void scheduleInstStop(ThreadID tid, Counter insts, const char *cause);
    void scheduleLoadStop(ThreadID tid, Counter loads, const char *cause);
    uint64_t getCurrentInstCount(ThreadID tid);
''')

    @classmethod
    def memory_mode(cls):
        """Which memory mode does this CPU require?"""
        return 'invalid'

    @classmethod
    def require_caches(cls):
        """Does the CPU model require caches?

        Some CPU models might make assumptions that require them to
        have caches.
        """
        return False

    @classmethod
    def support_take_over(cls):
        """Does the CPU model support CPU takeOverFrom?"""
        return False

    def takeOverFrom(self, old_cpu):
        self._ccObject.takeOverFrom(old_cpu._ccObject)


    system = Param.System(Parent.any, "system object")
    cpu_id = Param.Int(-1, "CPU identifier")
    socket_id = Param.Unsigned(0, "Physical Socket identifier")
    numThreads = Param.Unsigned(1, "number of HW thread contexts")

    function_trace = Param.Bool(False, "Enable function trace")
    function_trace_start = Param.Tick(0, "Tick to start function trace")

    checker = Param.BaseCPU(NULL, "checker CPU")

    do_checkpoint_insts = Param.Bool(True,
        "enable checkpoint pseudo instructions")
    do_statistics_insts = Param.Bool(True,
        "enable statistics pseudo instructions")

    profile = Param.Latency('0ns', "trace the kernel stack")
    do_quiesce = Param.Bool(True, "enable quiesce instructions")

    workload = VectorParam.Process([], "processes to run")

    if buildEnv['TARGET_ISA'] == 'sparc':
        dtb = Param.SparcTLB(SparcTLB(), "Data TLB")
        itb = Param.SparcTLB(SparcTLB(), "Instruction TLB")
        interrupts = VectorParam.SparcInterrupts(
                [], "Interrupt Controller")
        isa = VectorParam.SparcISA([ isa_class() ], "ISA instance")
    elif buildEnv['TARGET_ISA'] == 'alpha':
        dtb = Param.AlphaTLB(AlphaDTB(), "Data TLB")
        itb = Param.AlphaTLB(AlphaITB(), "Instruction TLB")
        interrupts = VectorParam.AlphaInterrupts(
                [], "Interrupt Controller")
        isa = VectorParam.AlphaISA([ isa_class() ], "ISA instance")
    elif buildEnv['TARGET_ISA'] == 'x86':
        dtb = Param.X86TLB(X86TLB(), "Data TLB")
        itb = Param.X86TLB(X86TLB(), "Instruction TLB")
        interrupts = VectorParam.X86LocalApic([], "Interrupt Controller")
        isa = VectorParam.X86ISA([ isa_class() ], "ISA instance")
    elif buildEnv['TARGET_ISA'] == 'mips':
        dtb = Param.MipsTLB(MipsTLB(), "Data TLB")
        itb = Param.MipsTLB(MipsTLB(), "Instruction TLB")
        interrupts = VectorParam.MipsInterrupts(
                [], "Interrupt Controller")
        isa = VectorParam.MipsISA([ isa_class() ], "ISA instance")
    elif buildEnv['TARGET_ISA'] == 'arm':
        dtb = Param.ArmTLB(ArmTLB(), "Data TLB")
        itb = Param.ArmTLB(ArmTLB(), "Instruction TLB")
        istage2_mmu = Param.ArmStage2MMU(ArmStage2IMMU(), "Stage 2 trans")
        dstage2_mmu = Param.ArmStage2MMU(ArmStage2DMMU(), "Stage 2 trans")
        interrupts = VectorParam.ArmInterrupts(
                [], "Interrupt Controller")
        isa = VectorParam.ArmISA([ isa_class() ], "ISA instance")
    elif buildEnv['TARGET_ISA'] == 'power':
        UnifiedTLB = Param.Bool(True, "Is this a Unified TLB?")
        dtb = Param.PowerTLB(PowerTLB(), "Data TLB")
        itb = Param.PowerTLB(PowerTLB(), "Instruction TLB")
        interrupts = VectorParam.PowerInterrupts(
                [], "Interrupt Controller")
        isa = VectorParam.PowerISA([ isa_class() ], "ISA instance")
    else:
        print "Don't know what TLB to use for ISA %s" % \
            buildEnv['TARGET_ISA']
        sys.exit(1)

    max_insts_all_threads = Param.Counter(0,
        "terminate when all threads have reached this inst count")
    max_insts_any_thread = Param.Counter(0,
        "terminate when any thread reaches this inst count")
    simpoint_start_insts = VectorParam.Counter([],
        "starting instruction counts of simpoints")
    max_loads_all_threads = Param.Counter(0,
        "terminate when all threads have reached this load count")
    max_loads_any_thread = Param.Counter(0,
        "terminate when any thread reaches this load count")
    progress_interval = Param.Frequency('0Hz',
        "frequency to print out the progress message")

    switched_out = Param.Bool(False,
        "Leave the CPU switched out after startup (used when switching " \
        "between CPU models)")

    tracer = Param.InstTracer(default_tracer, "Instruction tracer")

    icache_port = MasterPort("Instruction Port")
    dcache_port = MasterPort("Data Port")
    _cached_ports = ['icache_port', 'dcache_port']

    if buildEnv['TARGET_ISA'] in ['x86', 'arm']:
        _cached_ports += ["itb.walker.port", "dtb.walker.port"]

    _uncached_slave_ports = []
    _uncached_master_ports = []
    if buildEnv['TARGET_ISA'] == 'x86':
        _uncached_slave_ports += ["interrupts[0].pio",
                                  "interrupts[0].int_slave"]
        _uncached_master_ports += ["interrupts[0].int_master"]

    def createInterruptController(self):
        if buildEnv['TARGET_ISA'] == 'sparc':
            self.interrupts = [SparcInterrupts() for i in xrange(self.numThreads)]
        elif buildEnv['TARGET_ISA'] == 'alpha':
            self.interrupts = [AlphaInterrupts() for i in xrange(self.numThreads)]
        elif buildEnv['TARGET_ISA'] == 'x86':
            self.apic_clk_domain = DerivedClockDomain(clk_domain =
                                                      Parent.clk_domain,
                                                      clk_divider = 16)
            self.interrupts = [X86LocalApic(clk_domain = self.apic_clk_domain,
                                           pio_addr=0x2000000000000000)
                               for i in xrange(self.numThreads)]
            _localApic = self.interrupts
        elif buildEnv['TARGET_ISA'] == 'mips':
            self.interrupts = [MipsInterrupts() for i in xrange(self.numThreads)]
        elif buildEnv['TARGET_ISA'] == 'arm':
            self.interrupts = [ArmInterrupts() for i in xrange(self.numThreads)]
        elif buildEnv['TARGET_ISA'] == 'power':
            self.interrupts = [PowerInterrupts() for i in xrange(self.numThreads)]
        else:
            print "Don't know what Interrupt Controller to use for ISA %s" % \
                buildEnv['TARGET_ISA']
            sys.exit(1)

    def connectCachedPorts(self, bus):
        for p in self._cached_ports:
            exec('self.%s = bus.slave' % p)

    def connectUncachedPorts(self, bus):
        for p in self._uncached_slave_ports:
            exec('self.%s = bus.master' % p)
        for p in self._uncached_master_ports:
            exec('self.%s = bus.slave' % p)

    def connectAllPorts(self, cached_bus, uncached_bus = None):
        self.connectCachedPorts(cached_bus)
        if not uncached_bus:
            uncached_bus = cached_bus
        self.connectUncachedPorts(uncached_bus)

    def addPrivateSplitL1Caches(self, ic, dc, iwc = None, dwc = None):
        self.icache = ic
        self.dcache = dc
        self.icache_port = ic.cpu_side
        self.dcache_port = dc.cpu_side
        self._cached_ports = ['icache.mem_side', 'dcache.mem_side']
        if buildEnv['TARGET_ISA'] in ['x86', 'arm']:
            if iwc and dwc:
                self.itb_walker_cache = iwc
                self.dtb_walker_cache = dwc
                self.itb.walker.port = iwc.cpu_side
                self.dtb.walker.port = dwc.cpu_side
                self._cached_ports += ["itb_walker_cache.mem_side", \
                                       "dtb_walker_cache.mem_side"]
            else:
                self._cached_ports += ["itb.walker.port", "dtb.walker.port"]

            # Checker doesn't need its own tlb caches because it does
            # functional accesses only
            if self.checker != NULL:
                self._cached_ports += ["checker.itb.walker.port", \
                                       "checker.dtb.walker.port"]

    def addTwoLevelCacheHierarchy(self, ic, dc, l2c, iwc = None, dwc = None):
        self.addPrivateSplitL1Caches(ic, dc, iwc, dwc)
        self.toL2Bus = L2XBar()
        self.connectCachedPorts(self.toL2Bus)
        self.l2cache = l2c
        self.toL2Bus.master = self.l2cache.cpu_side
        self._cached_ports = ['l2cache.mem_side']

    def createThreads(self):
        self.isa = [ isa_class() for i in xrange(self.numThreads) ]
        if self.checker != NULL:
            self.checker.createThreads()

    def addCheckerCpu(self):
        pass
