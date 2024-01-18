# Copyright (c) 2022 The Regents of the University of California
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

from typing import (
    List,
    Optional,
)

from m5.objects import (
    BaseCPU,
    BaseMMU,
    PcCountTracker,
    PcCountTrackerManager,
    Port,
    Process,
)
from m5.params import PcCountPair

from ...isas import ISA
from ...utils.override import overrides
from ...utils.requires import requires
from .abstract_core import AbstractCore


class BaseCPUCore(AbstractCore):
    """
    An stdlib AbstractCore subclass which wraps a BaseCPU SimObject type.
    """

    def __init__(self, core: BaseCPU, isa: ISA):
        super().__init__()

        # There is some annoying redundancy here. The BaseCPU type already
        # defines the ISA, so here we are defining it twice. However, there
        # currently isn't a good way to get the ISA from the BaseCPU Type.
        #
        # TODO: Have some helper function to get the ISA from a BaseCPU type.
        # This may just be a cause of using `instanceof`:
        # e.g., `if instanceof(cpu, X86Cpu): return ISA.X86`.
        #
        requires(isa_required=isa)
        self._isa = isa

        self.core = core
        self.core.createThreads()

    def get_simobject(self) -> BaseCPU:
        return self.core

    @overrides(AbstractCore)
    def requires_send_evicts(self) -> bool:
        if self.get_isa() in (ISA.ARM, ISA.X86):
            # * The x86 `mwait`` instruction is built on top of coherence,
            #   therefore evictions must be sent from cache to the CPU Core.
            #
            # * The local exclusive monitor in ARM systems requires the sending
            #    of evictions from cache to the CPU Core.
            return True

        # The O3 model must keep the LSQ coherent with the caches.
        # The code below will check to see if the current base CPU is of the O3
        # type for the current ISA target (a bit ugly but it works).

        try:
            from m5.objects import BaseO3CPU

            return isinstance(self.get_simobject(), BaseO3CPU)
        except ImportError:
            # If, for whatever reason, the BaseO3CPU is not importable, then
            # the current core cannot be an an O3 CPU. We therefore return
            # False.
            return False

    @overrides(AbstractCore)
    def is_kvm_core(self) -> bool:
        try:
            from m5.objects import BaseKvmCPU

            return isinstance(self.core, BaseKvmCPU)
        except ImportError:
            # If importing BaseKvmCPU throws an exception then it's because
            # it's not compiled into the binary. If this is the case then this
            # can't be a KVM core.
            return False

    def get_isa(self) -> ISA:
        return self._isa

    @overrides(AbstractCore)
    def connect_icache(self, port: Port) -> None:
        self.core.icache_port = port

    @overrides(AbstractCore)
    def connect_dcache(self, port: Port) -> None:
        self.core.dcache_port = port

    @overrides(AbstractCore)
    def connect_walker_ports(self, port1: Port, port2: Port) -> None:
        if self.get_isa() == ISA.ARM:
            # Unlike X86 and RISCV MMU, the ARM MMU has two L1 TLB walker ports
            # named `walker` and `stage2_walker` for both data and instruction.
            # The gem5 standard library currently supports one TLB walker port
            # per cache level. Therefore, we are explicitly setting the walker
            # ports and not setting the stage2_walker ports for ARM systems.

            self.core.mmu.itb_walker.port = port1
            self.core.mmu.dtb_walker.port = port2
        else:
            self.core.mmu.connectWalkerPorts(port1, port2)

    @overrides(AbstractCore)
    def set_workload(self, process: Process) -> None:
        self.core.workload = process

    @overrides(AbstractCore)
    def set_switched_out(self, value: bool) -> None:
        self.core.switched_out = value

    @overrides(AbstractCore)
    def connect_interrupt(
        self,
        interrupt_requestor: Optional[Port] = None,
        interrupt_responce: Optional[Port] = None,
    ) -> None:
        # TODO: This model assumes that we will only create an interrupt
        # controller as we require it. Not sure how true this is in all cases.
        self.core.createInterruptController()

        if self.get_isa().value == ISA.X86.value:
            if interrupt_requestor != None:
                self.core.interrupts[0].pio = interrupt_requestor
                self.core.interrupts[0].int_responder = interrupt_requestor
            if interrupt_responce != None:
                self.core.interrupts[0].int_requestor = interrupt_responce

    @overrides(AbstractCore)
    def get_mmu(self) -> BaseMMU:
        return self.core.mmu

    @overrides(AbstractCore)
    def _set_simpoint(
        self, inst_starts: List[int], board_initialized: bool
    ) -> None:
        if board_initialized:
            self.core.scheduleSimpointsInstStop(sorted(set(inst_starts)))
        else:
            self.core.simpoint_start_insts = sorted(set(inst_starts))

    @overrides(AbstractCore)
    def _set_inst_stop_any_thread(
        self, inst: int, board_initialized: bool
    ) -> None:
        if board_initialized:
            self.core.scheduleInstStopAnyThread(inst)
        else:
            self.core.max_insts_any_thread = inst

    @overrides(AbstractCore)
    def add_pc_tracker_probe(
        self, target_pair: List[PcCountPair], manager: PcCountTrackerManager
    ) -> None:
        pair_tracker = PcCountTracker()
        pair_tracker.targets = target_pair
        pair_tracker.core = self.core
        pair_tracker.ptmanager = manager
        self.core.probeListener = pair_tracker
