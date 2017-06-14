# -*- mode:python -*-
# Copyright (c) 2009-2014, 2017 ARM Limited
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
# Authors: Matt Horsnell
#          Andreas Sandberg

from m5.defines import buildEnv
from m5.SimObject import *
from m5.params import *
from m5.params import isNullPointer
from m5.proxy import *

class ArmPMU(SimObject):
    type = 'ArmPMU'
    cxx_class = 'ArmISA::PMU'
    cxx_header = 'arch/arm/pmu.hh'

    cxx_exports = [
        PyBindMethod("addEventProbe"),
    ]

    # To prevent cycles in the configuration hierarchy, we don't keep
    # a list of supported events as a configuration param. Instead, we
    # keep them in a local list and register them using the
    # addEventProbe interface when other SimObjects register their
    # probe listeners.
    _deferred_event_types = []
    # Override the normal SimObject::regProbeListeners method and
    # register deferred event handlers.
    def regProbeListeners(self):
        for event_id, obj, name in self._deferred_event_types:
            self.getCCObject().addEventProbe(event_id, obj.getCCObject(), name)

        self.getCCObject().regProbeListeners()

    def addEventProbe(self, event_id, obj, *args):
        """Add a probe-based event to the PMU if obj is not None."""

        if obj is None:
            return

        for name in args:
            self._deferred_event_types.append((event_id, obj, name))

    def addArchEvents(self,
                      cpu=None,
                      itb=None, dtb=None,
                      icache=None, dcache=None,
                      l2cache=None):
        """Add architected events to the PMU.

        This method can be called multiple times with only a subset of
        the keyword arguments set. This enables event registration in
        configuration scripts to happen closer to the instantiation of
        the instrumented objects (e.g., the memory system) instead of
        a central point.

        CPU events should also be registered once per CPU that is
        sharing the PMU (e.g., when switching between CPU models).
        """

        bpred = cpu.branchPred if cpu and not isNullPointer(cpu.branchPred) \
            else None

        # 0x01: L1I_CACHE_REFILL
        self.addEventProbe(0x02, itb, "Refills")
        # 0x03: L1D_CACHE_REFILL
        # 0x04: L1D_CACHE
        self.addEventProbe(0x05, dtb, "Refills")
        self.addEventProbe(0x06, cpu, "RetiredLoads")
        self.addEventProbe(0x07, cpu, "RetiredStores")
        self.addEventProbe(0x08, cpu, "RetiredInsts")
        # 0x09: EXC_TAKEN
        # 0x0A: EXC_RETURN
        # 0x0B: CID_WRITE_RETIRED
        self.addEventProbe(0x0C, cpu, "RetiredBranches")
        # 0x0D: BR_IMMED_RETIRED
        # 0x0E: BR_RETURN_RETIRED
        # 0x0F: UNALIGEND_LDST_RETIRED
        self.addEventProbe(0x10, bpred, "Misses")
        self.addEventProbe(0x11, cpu, "Cycles")
        self.addEventProbe(0x12, bpred, "Branches")
        self.addEventProbe(0x13, cpu, "RetiredLoads", "RetiredStores")
        # 0x14: L1I_CACHE
        # 0x15: L1D_CACHE_WB
        # 0x16: L2D_CACHE
        # 0x17: L2D_CACHE_REFILL
        # 0x18: L2D_CACHE_WB
        # 0x19: BUS_ACCESS
        # 0x1A: MEMORY_ERROR
        # 0x1B: INST_SPEC
        # 0x1C: TTBR_WRITE_RETIRED
        # 0x1D: BUS_CYCLES
        # 0x1E: CHAIN
        # 0x1F: L1D_CACHE_ALLOCATE
        # 0x20: L2D_CACHE_ALLOCATE
        # 0x21: BR_RETIRED
        # 0x22: BR_MIS_PRED_RETIRED
        # 0x23: STALL_FRONTEND
        # 0x24: STALL_BACKEND
        # 0x25: L1D_TLB
        # 0x26: L1I_TLB
        # 0x27: L2I_CACHE
        # 0x28: L2I_CACHE_REFILL
        # 0x29: L3D_CACHE_ALLOCATE
        # 0x2A: L3D_CACHE_REFILL
        # 0x2B: L3D_CACHE
        # 0x2C: L3D_CACHE_WB
        # 0x2D: L2D_TLB_REFILL
        # 0x2E: L2I_TLB_REFILL
        # 0x2F: L2D_TLB
        # 0x30: L2I_TLB

    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    eventCounters = Param.Int(31, "Number of supported PMU counters")
    pmuInterrupt = Param.Int(68, "PMU GIC interrupt number")
