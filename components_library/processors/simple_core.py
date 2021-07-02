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

from components_library.runtime import get_runtime_isa
from components_library.processors.abstract_core import AbstractCore

from .cpu_types import CPUTypes
from ..isas import ISA
from ..utils.override import overrides

from m5.objects import (
    Port,
    AtomicSimpleCPU,
    DerivO3CPU,
    TimingSimpleCPU,
    X86KvmCPU,
    BaseCPU,
    Process,
)


class SimpleCore(AbstractCore):
    def __init__(self, cpu_type: CPUTypes, core_id: int):
        super(SimpleCore, self).__init__(cpu_type=cpu_type)

        if cpu_type == CPUTypes.ATOMIC:
            self.core = AtomicSimpleCPU(cpu_id=core_id)
        elif cpu_type == CPUTypes.O3:
            self.core = DerivO3CPU(cpu_id=core_id)
        elif cpu_type == CPUTypes.TIMING:
            self.core = TimingSimpleCPU(cpu_id=core_id)
        elif cpu_type == CPUTypes.KVM:
            self.core = X86KvmCPU(cpu_id=core_id)
        else:
            raise NotImplementedError

        self.core.createThreads()

    def get_simobject(self) -> BaseCPU:
        return self.core

    @overrides(AbstractCore)
    def connect_icache(self, port: Port) -> None:
        self.core.icache_port = port

    @overrides(AbstractCore)
    def connect_dcache(self, port: Port) -> None:
        self.core.dcache_port = port

    @overrides(AbstractCore)
    def connect_walker_ports(self, port1: Port, port2: Port) -> None:
        self.core.mmu.connectWalkerPorts(port1, port2)

    @overrides(AbstractCore)
    def set_workload(self, process: Process) -> None:
        self.core.workload = process

    @overrides(AbstractCore)
    def set_switched_out(self, value: bool) -> None:
        self.core.switched_out = value

    @overrides(AbstractCore)
    def connect_interrupt(
        self, interrupt_requestor: Port, interrupt_responce: Port
    ) -> None:

        # TODO: This model assumes that we will only create an interrupt
        # controller as we require it. Not sure how true this is in all cases.
        self.core.createInterruptController()

        if get_runtime_isa() == ISA.X86:
            self.core.interrupts[0].pio = interrupt_requestor
            self.core.interrupts[0].int_requestor = interrupt_responce
            self.core.interrupts[0].int_responder = interrupt_requestor
