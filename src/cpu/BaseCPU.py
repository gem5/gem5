# Copyright (c) 2005-2007 The Regents of The University of Michigan
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

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5 import build_env
from Bus import Bus
from InstTracer import InstTracer
from ExeTracer import ExeTracer
import sys

default_tracer = ExeTracer()

if build_env['TARGET_ISA'] == 'alpha':
    from AlphaTLB import AlphaDTB, AlphaITB
elif build_env['TARGET_ISA'] == 'sparc':
    from SparcTLB import SparcDTB, SparcITB
elif build_env['TARGET_ISA'] == 'x86':
    from X86TLB import X86DTB, X86ITB
elif build_env['TARGET_ISA'] == 'mips':
    from MipsTLB import MipsDTB, MipsITB

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
    else:
        workload = VectorParam.Process("processes to run")

    if build_env['TARGET_ISA'] == 'sparc':
        dtb = Param.SparcDTB(SparcDTB(), "Data TLB")
        itb = Param.SparcITB(SparcITB(), "Instruction TLB")
    elif build_env['TARGET_ISA'] == 'alpha':
        dtb = Param.AlphaDTB(AlphaDTB(), "Data TLB")
        itb = Param.AlphaITB(AlphaITB(), "Instruction TLB")
    elif build_env['TARGET_ISA'] == 'x86':
        dtb = Param.X86DTB(X86DTB(), "Data TLB")
        itb = Param.X86ITB(X86ITB(), "Instruction TLB")
    elif build_env['TARGET_ISA'] == 'mips':
        dtb = Param.MipsDTB(MipsDTB(), "Data TLB")
        itb = Param.MipsITB(MipsITB(), "Instruction TLB")
    else:
        print "Don't know what TLB to use for ISA %s" % \
            build_env['TARGET_ISA']
        sys.exit(1)

    max_insts_all_threads = Param.Counter(0,
        "terminate when all threads have reached this inst count")
    max_insts_any_thread = Param.Counter(0,
        "terminate when any thread reaches this inst count")
    max_loads_all_threads = Param.Counter(0,
        "terminate when all threads have reached this load count")
    max_loads_any_thread = Param.Counter(0,
        "terminate when any thread reaches this load count")
    progress_interval = Param.Tick(0,
        "interval to print out the progress message")

    defer_registration = Param.Bool(False,
        "defer registration with system (for sampling)")

    clock = Param.Clock('1t', "clock speed")
    phase = Param.Latency('0ns', "clock phase")

    tracer = Param.InstTracer(default_tracer, "Instruction tracer")

    _mem_ports = []

    if build_env['TARGET_ISA'] == 'x86' and build_env['FULL_SYSTEM']:
        itb.walker_port = Port("ITB page table walker port")
        dtb.walker_port = Port("ITB page table walker port")
        _mem_ports = ["itb.walker_port", "dtb.walker_port"]

    def connectMemPorts(self, bus):
        for p in self._mem_ports:
            if p != 'physmem_port':
                exec('self.%s = bus.port' % p)

    def addPrivateSplitL1Caches(self, ic, dc):
        assert(len(self._mem_ports) < 6)
        self.icache = ic
        self.dcache = dc
        self.icache_port = ic.cpu_side
        self.dcache_port = dc.cpu_side
        self._mem_ports = ['icache.mem_side', 'dcache.mem_side']
        if build_env['TARGET_ISA'] == 'x86' and build_env['FULL_SYSTEM']:
            self._mem_ports += ["itb.walker_port", "dtb.walker_port"]

    def addTwoLevelCacheHierarchy(self, ic, dc, l2c):
        self.addPrivateSplitL1Caches(ic, dc)
        self.toL2Bus = Bus()
        self.connectMemPorts(self.toL2Bus)
        self.l2cache = l2c
        self.l2cache.cpu_side = self.toL2Bus.port
        self._mem_ports = ['l2cache.mem_side']
