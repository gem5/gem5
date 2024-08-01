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

from typing import (
    List,
    Optional,
)

from m5.objects.XBar import IOXBar
from m5.params import (
    AddrRange,
    Port,
)

from ...utils.override import overrides
from ..cachehierarchies.abstract_cache_hierarchy import AbstractCacheHierarchy
from ..memory.abstract_memory_system import AbstractMemorySystem
from ..processors.abstract_generator import AbstractGenerator
from .abstract_board import AbstractBoard
from .abstract_system_board import AbstractSystemBoard


class TestBoard(AbstractSystemBoard):
    """This is a Testing Board used to run traffic generators on a simple
    architecture.

    To work as a traffic generator board, pass a generator as a processor.

    This board does not require a cache hierarchy (it can be ``none``) in which
    case the processor (generator) will be directly connected to the memory.
    The clock frequency is only used if there is a cache hierarchy or when
    using the GUPS generators.
    """

    def __init__(
        self,
        clk_freq: str,
        generator: AbstractGenerator,
        memory: AbstractMemorySystem,
        cache_hierarchy: Optional[AbstractCacheHierarchy],
    ):
        super().__init__(
            clk_freq=clk_freq,  # Only used if cache hierarchy or GUPS-gen
            processor=generator,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )
        self._set_fullsystem(False)

    @overrides(AbstractSystemBoard)
    def _setup_board(self) -> None:
        pass

    @overrides(AbstractSystemBoard)
    def has_io_bus(self) -> bool:
        return False

    @overrides(AbstractSystemBoard)
    def get_io_bus(self) -> IOXBar:
        raise NotImplementedError(
            "The TestBoard does not have an IO Bus. "
            "Use `has_io_bus()` to check this."
        )

    @overrides(AbstractSystemBoard)
    def has_dma_ports(self) -> bool:
        return False

    @overrides(AbstractSystemBoard)
    def get_dma_ports(self) -> List[Port]:
        raise NotImplementedError(
            "The TestBoard does not have DMA Ports. "
            "Use `has_dma_ports()` to check this."
        )

    @overrides(AbstractSystemBoard)
    def has_coherent_io(self) -> bool:
        return False

    @overrides(AbstractSystemBoard)
    def get_mem_side_coherent_io_port(self):
        raise NotImplementedError(
            "SimpleBoard does not have any I/O ports. Use has_coherent_io to "
            "check this."
        )

    @overrides(AbstractSystemBoard)
    def _setup_memory_ranges(self) -> None:
        memory = self.get_memory()

        # The simple board just has one memory range that is the size of the
        # memory.
        self.mem_ranges = [AddrRange(memory.get_size())]
        memory.set_memory_range(self.mem_ranges)

    @overrides(AbstractBoard)
    def _connect_things(self) -> None:
        super()._connect_things()

        if not self.get_cache_hierarchy():
            # If we have no caches, then there must be a one-to-one
            # connection between the generators and the memories.
            assert len(self.get_processor().get_cores()) == 1
            assert len(self.get_memory().get_mem_ports()) == 1
            self.get_processor().get_cores()[0].connect_dcache(
                self.get_memory().get_mem_ports()[0][1]
            )
