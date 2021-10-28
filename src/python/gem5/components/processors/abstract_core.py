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
from typing import Optional
from .cpu_types import CPUTypes
from ...utils.requires import requires

from m5.objects import BaseMMU, Port, SubSystem


class AbstractCore(SubSystem):
    __metaclass__ = ABCMeta

    def __init__(self, cpu_type: CPUTypes):
        super().__init__()
        if cpu_type == CPUTypes.KVM:
            requires(kvm_required=True)
        self._cpu_type = cpu_type

    def get_type(self) -> CPUTypes:
        return self._cpu_type

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
        self, interrupt_requestor: Optional[Port] = None,
        interrupt_responce: Optional[Port] = None
    ) -> None:
        """ Connect the core interrupts to the interrupt controller

        This function is usually called from the cache hierarchy since the
        optional ports can be implemented as cache ports.
        """
        raise NotImplementedError

    @abstractmethod
    def get_mmu(self) -> BaseMMU:
        """ Return the MMU for this core.

        This is used in the board to setup system-specific MMU settings.
        """
        raise NotImplementedError
