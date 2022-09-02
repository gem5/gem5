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

from abc import ABCMeta, abstractmethod
from typing import Optional, List

from ...isas import ISA

from m5.objects import BaseMMU, Port, SubSystem


class AbstractCore(SubSystem):
    __metaclass__ = ABCMeta

    def __init__(self):
        super().__init__()

    @abstractmethod
    def get_isa(self) -> ISA:
        raise NotImplementedError

    @abstractmethod
    def requires_send_evicts(self) -> bool:
        """True if the CPU model or ISA requires sending evictions from caches
        to the CPU. Scenarios warrant forwarding evictions to the CPU:
        1. The O3 model must keep the LSQ coherent with the caches
        2. The x86 mwait instruction is built on top of coherence
        3. The local exclusive monitor in ARM systems
        """
        return False

    @abstractmethod
    def is_kvm_core(self) -> bool:
        """
        KVM cores need setup differently than other cores. Frequently it's
        useful to know whether a core is a KVM core or not. This function helps
        with this.
        """
        raise NotImplementedError

    @abstractmethod
    def connect_icache(self, port: Port) -> None:
        """
        This function should connect the response port from the instruction
        cache to the right request port on the core.

        :param port: The response port from the icache to connect to.
        """
        raise NotImplementedError

    @abstractmethod
    def connect_dcache(self, port: Port) -> None:
        """
        This function should connect the response port from the data cache to
        the right request port on the core.

        :param port: The response port from the icache to connect to.
        """
        raise NotImplementedError

    @abstractmethod
    def connect_walker_ports(self, port1: Port, port2: Port) -> None:
        """
        Connect the response port from itb and dtb to their respective request
        ports in the core.

        :param port1: The response port from itb walker to connect to.
        :param port2: The response port from dtb walker to connect to.
        """
        raise NotImplementedError

    @abstractmethod
    def set_workload(self, process: "Process") -> None:
        raise NotImplementedError

    @abstractmethod
    def set_switched_out(self, value: bool) -> None:
        raise NotImplementedError

    @abstractmethod
    def connect_interrupt(
        self,
        interrupt_requestor: Optional[Port] = None,
        interrupt_responce: Optional[Port] = None,
    ) -> None:
        """Connect the core interrupts to the interrupt controller

        This function is usually called from the cache hierarchy since the
        optional ports can be implemented as cache ports.
        """
        raise NotImplementedError

    @abstractmethod
    def get_mmu(self) -> BaseMMU:
        """Return the MMU for this core.

        This is used in the board to setup system-specific MMU settings.
        """
        raise NotImplementedError

    @abstractmethod
    def set_simpoint(self, inst_starts: List[int], init: bool) -> None:
        """Schedule simpoint exit events for the core.

        This is used to raise SIMPOINT_BEGIN exit events in the gem5 standard
        library.

        :param inst_starts: a list of SimPoints starting instructions
        :param init: if it is True, the starting instructions will be scheduled
        at the init stage of the core, else, the starting insructions will be
        scheduled during the simulation
        """
        raise NotImplementedError("This core type does not support simpoints")

    @abstractmethod
    def set_inst_stop_any_thread(self, inst: int, init: bool) -> None:
        """Schedule an exit event when any thread in this core reaches the
        given number of instructions.

        This is used to raise MAX_INSTS exit event in the gem5 standard library

        :param inst: a number of instructions
        :param init: if it is True, the exit event will be scheduled at the
        init stage of the core, else, it will be scheduled during the
        simulation
        """
        raise NotImplementedError("This core type does not support MAX_INSTS")
