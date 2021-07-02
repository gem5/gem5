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

from m5.objects import (
    SrcClockDomain,
    VoltageDomain,
    Process,
    SEWorkload,
    IOXBar,
    Port,
    ClockDomain,
)

from .abstract_board import AbstractBoard
from .mem_mode import MemMode, mem_mode_to_string
from ..processors.abstract_processor import AbstractProcessor
from ..memory.abstract_memory_system import AbstractMemorySystem
from ..cachehierarchies.abstract_cache_hierarchy import AbstractCacheHierarchy
from ..utils.override import overrides

from typing import List


class SimpleBoard(AbstractBoard):
    """
    This is an incredibly simple system. It contains no I/O, and will work only
    with a classic cache hierarchy setup.

    **Limitations**
    * Only supports SE mode

    You can run a binary executable via the `set_workload` function.
    """

    def __init__(
        self,
        clk_freq: str,
        processor: AbstractProcessor,
        memory: AbstractMemorySystem,
        cache_hierarchy: AbstractCacheHierarchy,
        exit_on_work_items: bool = False,
    ) -> None:
        super(SimpleBoard, self).__init__(
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

        # Set up the clock domain and the voltage domain.
        self.clk_domain = SrcClockDomain()
        self.clk_domain.clock = clk_freq
        self.clk_domain.voltage_domain = VoltageDomain()

        self.mem_ranges = memory.get_memory_ranges()

        self.exit_on_work_items = exit_on_work_items

    @overrides(AbstractBoard)
    def get_clock_domain(self) -> ClockDomain:
        return self.clk_domain

    @overrides(AbstractBoard)
    def connect_system_port(self, port: Port) -> None:
        self.system_port = port

    @overrides(AbstractBoard)
    def set_mem_mode(self, mem_mode: MemMode) -> None:
        self.mem_mode = mem_mode_to_string(mem_mode=mem_mode)

    @overrides(AbstractBoard)
    def connect_things(self) -> None:
        # Incorporate the cache hierarchy for the motherboard.
        self.get_cache_hierarchy().incorporate_cache(self)

        # Incorporate the processor into the motherboard.
        self.get_processor().incorporate_processor(self)

        # Incorporate the memory into the motherboard.
        self.get_memory().incorporate_memory(self)

    @overrides(AbstractBoard)
    def has_io_bus(self) -> bool:
        return False

    @overrides(AbstractBoard)
    def get_io_bus(self) -> IOXBar:
        raise NotImplementedError(
            "SimpleBoard does not have an IO Bus. "
            "Use `has_io_bus()` to check this."
        )

    @overrides(AbstractBoard)
    def has_dma_ports(self) -> bool:
        return False

    @overrides(AbstractBoard)
    def get_dma_ports(self) -> List[Port]:
        raise NotImplementedError(
            "SimpleBoard does not have DMA Ports. "
            "Use `has_dma_ports()` to check this."
        )

    def set_workload(self, binary: str) -> None:
        """Set up the system to run a specific binary.

        **Limitations**
        * Only supports single threaded applications
        * Dynamically linked executables are partially supported when the host
          ISA and the simulated ISA are the same.

        :param binary: The path on the *host* to the binary to run in gem5.
        """

        self.workload = SEWorkload.init_compatible(binary)

        process = Process()
        process.cmd = [binary]
        self.get_processor().get_cores()[0].set_workload(process)
