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

"""Single channel "generic" DDR memory controllers
"""

from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem
from ...utils.override import overrides

from m5.objects import AddrRange, DRAMInterface, MemCtrl, Port
from m5.util.convert import toMemorySize

from typing import List, Sequence, Tuple, Type, Optional


class SingleChannelMemory(AbstractMemorySystem):
    """A simple implementation of a single channel memory system

    This class can take a DRAM Interface as a parameter to model many different
    DDR memory systems.
    """

    def __init__(
        self,
        dram_interface_class: Type[DRAMInterface],
        size: Optional[str] = None,
    ):
        """
        :param dram_interface_class: The DRAM interface type to create with
            this memory controller
        :param size: Optionally specify the size of the DRAM controller's
            address space. By default, it starts at 0 and ends at the size of
            the DRAM device specified
        """
        super().__init__()

        self._dram = dram_interface_class()
        if size:
            self._size = toMemorySize(size)
        else:
            self._size = self._get_dram_size(self._dram)
        self.mem_ctrl = MemCtrl(dram=self._dram)

    def _get_dram_size(self, dram: DRAMInterface) -> int:
        return (
            dram.device_size.value
            * dram.devices_per_rank.value
            * dram.ranks_per_channel.value
        )

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        pass

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Tuple[Sequence[AddrRange], Port]:
        return [(self._dram.range, self.mem_ctrl.port)]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> List[MemCtrl]:
        return [self.mem_ctrl]

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        if len(ranges) != 1 or ranges[0].size() != self._size:
            print(ranges[0].size())
            raise Exception(
                "Single channel memory controller requires a single range "
                "which matches the memory's size."
            )
        self.mem_ctrl.dram.range = ranges[0]


from .dram_interfaces.ddr3 import DDR3_1600_8x8, DDR3_2133_8x8
from .dram_interfaces.ddr4 import DDR4_2400_8x8
from .dram_interfaces.lpddr3 import LPDDR3_1600_1x32
from .dram_interfaces.hbm import HBM_1000_4H_1x128

# Enumerate all of the different DDR memory systems we support
def SingleChannelDDR3_1600(size: Optional[str] = None) -> AbstractMemorySystem:
    """
    A single channel memory system using a single DDR3_1600_8x8 based DIMM
    """
    return SingleChannelMemory(DDR3_1600_8x8, size)


def SingleChannelDDR3_2133(size: Optional[str] = None) -> AbstractMemorySystem:
    """
    A single channel memory system using a single DDR3_2133_8x8 based DIMM
    """
    return SingleChannelMemory(DDR3_2133_8x8, size)


def SingleChannelDDR4_2400(size: Optional[str] = None) -> AbstractMemorySystem:
    """
    A single channel memory system using a single DDR4_2400_8x8 based DIMM
    """
    return SingleChannelMemory(DDR4_2400_8x8, size)


def SingleChannelLPDDR3_1600(
    size: Optional[str] = None,
) -> AbstractMemorySystem:
    return SingleChannelMemory(LPDDR3_1600_1x32, size)


def SingleChannelHBM(size: Optional[str] = None) -> AbstractMemorySystem:
    return SingleChannelMemory(HBM_1000_4H_1x128, size)
