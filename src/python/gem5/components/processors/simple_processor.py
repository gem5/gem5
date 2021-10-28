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


from ...utils.override import overrides
from ..boards.mem_mode import MemMode
from ..processors.simple_core import SimpleCore

from m5.util import warn

from .abstract_processor import AbstractProcessor
from .cpu_types import CPUTypes
from ..boards.abstract_board import AbstractBoard


class SimpleProcessor(AbstractProcessor):
    """
    A SimpeProcessor contains a number of cores of a a single CPUType.
    """

    def __init__(self, cpu_type: CPUTypes, num_cores: int) -> None:
        super().__init__(
            cores=self._create_cores(
                cpu_type=cpu_type,
                num_cores=num_cores,
            )
        )

        self._cpu_type = cpu_type
        if self._cpu_type == CPUTypes.KVM:
            from m5.objects import KvmVM

            self.kvm_vm = KvmVM()

    def _create_cores(self, cpu_type: CPUTypes, num_cores: int):
        return [
            SimpleCore(cpu_type=cpu_type, core_id=i) for i in range(num_cores)
        ]

    @overrides(AbstractProcessor)
    def incorporate_processor(self, board: AbstractBoard) -> None:
        if self._cpu_type == CPUTypes.KVM:
            board.kvm_vm = self.kvm_vm

        # Set the memory mode.
        if self._cpu_type == CPUTypes.TIMING or self._cpu_type == CPUTypes.O3:
            board.set_mem_mode(MemMode.TIMING)
        elif self._cpu_type == CPUTypes.KVM:
            board.set_mem_mode(MemMode.ATOMIC_NONCACHING)
        elif self._cpu_type == CPUTypes.ATOMIC:
            if board.get_cache_hierarchy().is_ruby():
                warn(
                    "Using an atomic core with Ruby will result in "
                    "'atomic_noncaching' memory mode. This will skip caching "
                    "completely."
                )
            else:
                board.set_mem_mode(MemMode.ATOMIC)
        else:
            raise NotImplementedError

        if self._cpu_type == CPUTypes.KVM:
            # To get the KVM CPUs to run on different host CPUs
            # Specify a different event queue for each CPU
            for i, core in enumerate(self.cores):
                for obj in core.get_simobject().descendants():
                    obj.eventq_index = 0
                core.get_simobject().eventq_index = i + 1
