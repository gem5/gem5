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


from typing import List

from m5.objects import (
    BaseAtomicSimpleCPU,
    BaseMinorCPU,
    BaseNonCachingSimpleCPU,
    BaseO3CPU,
    BaseTimingSimpleCPU,
)
from m5.util import warn

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from ..boards.mem_mode import MemMode
from .abstract_processor import AbstractProcessor
from .base_cpu_core import BaseCPUCore


class BaseCPUProcessor(AbstractProcessor):
    """
    A processor constructed from a List of BaseCPUCores.

    This gives gem5 stdlib users a way to create processors containing BaseCPU
    SimObjects. While SimpleProcessor does this by-proxy (the user simply
    specifies the desires CPUType and ISA and the correct BaseCPU
    instantiation is chosen), this Processor allows a more raw passing
    of BaseCPU objects.

    Disclaimer
    ----------

    Multiple cores comprising of different BaseCPU types has not been tested
    and is not officially supported.
    """

    def __init__(self, cores: List[BaseCPUCore]):
        super().__init__(cores=cores)

        if any(core.is_kvm_core() for core in self.get_cores()):
            from m5.objects import KvmVM

            self.kvm_vm = KvmVM()

    @overrides(AbstractProcessor)
    def incorporate_processor(self, board: AbstractBoard) -> None:
        if any(core.is_kvm_core() for core in self.get_cores()):
            board.kvm_vm = self.kvm_vm
            # To get the KVM CPUs to run on different host CPUs
            # Specify a different event queue for each CPU
            for i, core in enumerate(self.cores):
                for obj in core.get_simobject().descendants():
                    obj.eventq_index = 0
                core.get_simobject().eventq_index = i + 1
            board.set_mem_mode(MemMode.ATOMIC_NONCACHING)
        elif isinstance(
            self.cores[0].get_simobject(),
            (BaseTimingSimpleCPU, BaseO3CPU, BaseMinorCPU),
        ):
            board.set_mem_mode(MemMode.TIMING)
        elif isinstance(
            self.cores[0].get_simobject(), BaseNonCachingSimpleCPU
        ):
            board.set_mem_mode(MemMode.ATOMIC_NONCACHING)
        elif isinstance(self.cores[0].get_simobject(), BaseAtomicSimpleCPU):
            if board.get_cache_hierarchy().is_ruby():
                warn(
                    "Using an atomic core with Ruby will result in "
                    "'atomic_noncaching' memory mode. This will skip caching "
                    "completely."
                )
                board.set_mem_mode(MemMode.ATOMIC_NONCACHING)
            else:
                board.set_mem_mode(MemMode.ATOMIC)
        else:
            raise NotImplementedError
