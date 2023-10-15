# Copyright (c) 2012-2013, 2015-2017 ARM Limited
# Copyright (c) 2020 Barkhausen Institut
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

import sys

from m5.SimObject import *
from m5.defines import buildEnv
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *

from m5.objects.ClockedObject import ClockedObject
from m5.objects.XBar import L2XBar
from m5.objects.InstTracer import InstTracer
from m5.objects.CPUTracers import ExeTracer
from m5.objects.SubSystem import SubSystem
from m5.objects.ClockDomain import *
from m5.objects.Platform import Platform
from m5.objects.ResetPort import ResetResponsePort

default_tracer = ExeTracer()


class BaseCPU(ClockedObject):
    type = "BaseCPU"
    abstract = True
    cxx_header = "cpu/base.hh"
    cxx_class = "gem5::BaseCPU"

    cxx_exports = [
        PyBindMethod("switchOut"),
        PyBindMethod("takeOverFrom"),
        PyBindMethod("switchedOut"),
        PyBindMethod("flushTLBs"),
        PyBindMethod("totalInsts"),
        PyBindMethod("scheduleInstStop"),
        PyBindMethod("getCurrentInstCount"),
        PyBindMethod("scheduleSimpointsInstStop"),
        PyBindMethod("scheduleInstStopAnyThread"),
    ]

    @classmethod
    def memory_mode(cls):
        """Which memory mode does this CPU require?"""
        return "invalid"

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
    pwr_gating_latency = Param.Cycles(
        300,
        "Latency to enter power gating state when all contexts are suspended",
    )

    power_gating_on_idle = Param.Bool(
        False,
        "Control whether the core goes "
        "to the OFF power state after all thread are disabled for "
        "pwr_gating_latency cycles",
    )

    function_trace = Param.Bool(False, "Enable function trace")
    function_trace_start = Param.Tick(0, "Tick to start function trace")

    checker = Param.BaseCPU(NULL, "checker CPU")

    syscallRetryLatency = Param.Cycles(10000, "Cycles to wait until retry")

    do_checkpoint_insts = Param.Bool(
        True, "enable checkpoint pseudo instructions"
    )
    do_statistics_insts = Param.Bool(
        True, "enable statistics pseudo instructions"
    )

    workload = VectorParam.Process([], "processes to run")

    mmu = Param.BaseMMU(NULL, "CPU memory management unit")
    interrupts = VectorParam.BaseInterrupts([], "Interrupt Controller")
    isa = VectorParam.BaseISA([], "ISA instance")
    decoder = VectorParam.InstDecoder([], "Decoder instance")

    max_insts_all_threads = Param.Counter(
        0, "terminate when all threads have reached this inst count"
    )
    max_insts_any_thread = Param.Counter(
        0, "terminate when any thread reaches this inst count"
    )
    simpoint_start_insts = VectorParam.Counter(
        [], "starting instruction counts of simpoints"
    )
    progress_interval = Param.Frequency(
        "0Hz", "frequency to print out the progress message"
    )

    switched_out = Param.Bool(
        False,
        "Leave the CPU switched out after startup (used when switching "
        "between CPU models)",
    )

    model_reset = ResetResponsePort("Generic reset for the CPU")

    tracer = Param.InstTracer(default_tracer, "Instruction tracer")

    icache_port = RequestPort("Instruction Port")
    dcache_port = RequestPort("Data Port")
    _cached_ports = ["icache_port", "dcache_port"]

    _uncached_interrupt_response_ports = []
    _uncached_interrupt_request_ports = []

    def createInterruptController(self):
        self.interrupts = [
            self.ArchInterrupts() for i in range(self.numThreads)
        ]

    def connectCachedPorts(self, in_ports):
        for p in self._cached_ports:
            exec(f"self.{p} = in_ports")

    def connectUncachedPorts(self, in_ports, out_ports):
        for p in self._uncached_interrupt_response_ports:
            exec(f"self.{p} = out_ports")
        for p in self._uncached_interrupt_request_ports:
            exec(f"self.{p} = in_ports")

    def connectAllPorts(self, cached_in, uncached_in, uncached_out):
        self.connectCachedPorts(cached_in)
        self.connectUncachedPorts(uncached_in, uncached_out)

    def connectBus(self, bus):
        self.connectAllPorts(
            bus.cpu_side_ports, bus.cpu_side_ports, bus.mem_side_ports
        )

    def addPrivateSplitL1Caches(self, ic, dc, iwc=None, dwc=None):
        self.icache = ic
        self.dcache = dc
        self.icache_port = ic.cpu_side
        self.dcache_port = dc.cpu_side
        self._cached_ports = ["icache.mem_side", "dcache.mem_side"]
        if iwc and dwc:
            self.itb_walker_cache = iwc
            self.dtb_walker_cache = dwc
            self.mmu.connectWalkerPorts(iwc.cpu_side, dwc.cpu_side)
            self._cached_ports += [
                "itb_walker_cache.mem_side",
                "dtb_walker_cache.mem_side",
            ]
        else:
            self._cached_ports += self.ArchMMU.walkerPorts()

        # Checker doesn't need its own tlb caches because it does
        # functional accesses only
        if self.checker != NULL:
            self._cached_ports += [
                "checker." + port for port in self.ArchMMU.walkerPorts()
            ]

    def addTwoLevelCacheHierarchy(
        self, ic, dc, l2c, iwc=None, dwc=None, xbar=None
    ):
        self.addPrivateSplitL1Caches(ic, dc, iwc, dwc)
        self.toL2Bus = xbar if xbar else L2XBar()
        self.connectCachedPorts(self.toL2Bus.cpu_side_ports)
        self.l2cache = l2c
        self.toL2Bus.mem_side_ports = self.l2cache.cpu_side
        self._cached_ports = ["l2cache.mem_side"]

    def createThreads(self):
        # If no ISAs have been created, assume that the user wants the
        # default ISA.
        if len(self.isa) == 0:
            self.isa = [self.ArchISA() for i in range(self.numThreads)]
        else:
            if len(self.isa) != int(self.numThreads):
                raise RuntimeError(
                    "Number of ISA instances doesn't match thread count"
                )
        if len(self.decoder) != 0:
            raise RuntimeError("Decoders should not be set up manually")
        self.decoder = list([self.ArchDecoder(isa=isa) for isa in self.isa])
        if self.checker != NULL:
            self.checker.createThreads()

    def addCheckerCpu(self):
        pass

    def createPhandleKey(self, thread):
        # This method creates a unique key for this cpu as a function of a
        # certain thread
        return "CPU-%d-%d-%d" % (self.socket_id, self.cpu_id, thread)

    # Generate simple CPU Device Tree structure
    def generateDeviceTree(self, state):
        """Generate cpu nodes for each thread and the corresponding part of the
        cpu-map node. Note that this implementation does not support clusters
        of clusters. Note that GEM5 is not compatible with the official way of
        numbering cores as defined in the Device Tree documentation. Where the
        cpu_id needs to reset to 0 for each cluster by specification, GEM5
        expects the cpu_id to be globally unique and incremental. This
        generated node adheres the GEM5 way of doing things."""
        if bool(self.switched_out):
            return

        cpus_node = FdtNode("cpus")
        cpus_node.append(state.CPUCellsProperty())
        # Special size override of 0
        cpus_node.append(FdtPropertyWords("#size-cells", [0]))

        # Generate cpu nodes
        for i in range(int(self.numThreads)):
            reg = (int(self.socket_id) << 8) + int(self.cpu_id) + i
            node = FdtNode(f"cpu@{reg:x}")
            node.append(FdtPropertyStrings("device_type", "cpu"))
            node.appendCompatible(["gem5,arm-cpu"])
            node.append(FdtPropertyWords("reg", state.CPUAddrCells(reg)))
            platform, found = self.system.unproxy(self).find_any(Platform)
            if found:
                platform.annotateCpuDeviceNode(node, state)
            else:
                warn(
                    "Platform not found for device tree generation; "
                    "system or multiple CPUs may not start"
                )

            freq = int(self.clk_domain.unproxy(self).clock[0].frequency)
            node.append(FdtPropertyWords("clock-frequency", freq))

            # Unique key for this CPU
            phandle_key = self.createPhandleKey(i)
            node.appendPhandle(phandle_key)
            cpus_node.append(node)

        yield cpus_node

        # Generate nodes from the BaseCPU children (hence under the root node,
        # and don't add them as subnode). Please note: this is mainly needed
        # for the ISA class, to generate the PMU entry in the DTB.
        yield from self.recurseDeviceTree(state)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.power_state.possible_states = ["ON", "CLK_GATED", "OFF"]

        self._cached_ports = self._cached_ports + self.ArchMMU.walkerPorts()

        # Practically speaking, these ports will exist on the x86 interrupt
        # controller class.
        if "pio" in self.ArchInterrupts._ports:
            self._uncached_interrupt_response_ports = (
                self._uncached_interrupt_response_ports + ["interrupts[0].pio"]
            )
        if "int_responder" in self.ArchInterrupts._ports:
            self._uncached_interrupt_response_ports = (
                self._uncached_interrupt_response_ports
                + ["interrupts[0].int_responder"]
            )
        if "int_requestor" in self.ArchInterrupts._ports:
            self._uncached_interrupt_request_ports = (
                self._uncached_interrupt_request_ports
                + ["interrupts[0].int_requestor"]
            )
