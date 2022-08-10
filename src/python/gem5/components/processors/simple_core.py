# Copyright (c) 2021 The Regents of the University of California
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

from typing import Optional
from ...utils.requires import requires
from ..processors.abstract_core import AbstractCore

from .cpu_types import CPUTypes
from ...isas import ISA
from ...runtime import get_runtime_isa
from ...utils.override import overrides
from ...utils.requires import requires

from m5.objects import BaseMMU, Port, BaseCPU, Process


class SimpleCore(AbstractCore):
    """
    A SimpleCore instantiates a core based on the CPUType enum pass. The
    SimpleCore creates a single SimObject of that type.
    """

    def __init__(
        self, cpu_type: CPUTypes, core_id: int, isa: Optional[ISA] = None
    ):
        super().__init__()

        self._cpu_type = cpu_type
        if cpu_type == CPUTypes.KVM:
            requires(kvm_required=True)

        if isa:
            requires(isa_required=isa)
            self._isa = isa
        else:
            self._isa = get_runtime_isa()
        self.core = AbstractCore.cpu_simobject_factory(
            isa=self._isa, cpu_type=cpu_type, core_id=core_id
        )
        self.core.createThreads()

    def get_simobject(self) -> BaseCPU:
        return self.core

    def get_type(self) -> CPUTypes:
        return self._cpu_type

    @overrides(AbstractCore)
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

        if self.get_isa() == ISA.X86:
            if interrupt_requestor != None:
                self.core.interrupts[0].pio = interrupt_requestor
                self.core.interrupts[0].int_responder = interrupt_requestor
            if interrupt_responce != None:
                self.core.interrupts[0].int_requestor = interrupt_responce

    @overrides(AbstractCore)
    def get_mmu(self) -> BaseMMU:
        return self.core.mmu
