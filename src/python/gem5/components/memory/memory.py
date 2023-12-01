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

""" Channeled "generic" DDR memory controllers
"""

from math import log
from typing import (
    List,
    Optional,
    Sequence,
    Tuple,
    Type,
    Union,
)

from m5.objects import (
    AddrRange,
    DRAMInterface,
    MemCtrl,
    Port,
)
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem


def _try_convert(val, cls):
    try:
        return cls(val)
    except:
        raise Exception(f"Could not convert {val} to {cls}")


def _isPow2(num):
    log_num = int(log(num, 2))
    if 2**log_num != num:
        return False
    else:
        return True


class ChanneledMemory(AbstractMemorySystem):
    """A class to implement multi-channel memory system

    This class can take a DRAM Interface as a parameter to model a multi
    channel DDR DRAM memory system.
    """

    def __init__(
        self,
        dram_interface_class: Type[DRAMInterface],
        num_channels: Union[int, str],
        interleaving_size: Union[int, str],
        size: Optional[str] = None,
        addr_mapping: Optional[str] = None,
    ) -> None:
        """
        :param dram_interface_class: The DRAM interface type to create with
                                     this memory controller.
        :param num_channels: The number of channels that needs to be
                             simulated.
        :param size: Optionally specify the size of the DRAM controller's
                     address space. By default, it starts at 0 and ends at
                     the size of the DRAM device specified.
        :param addr_mapping: Defines the address mapping scheme to be used.
                             If ``None``, it is defaulted to ``addr_mapping`` from
                             ``dram_interface_class``.
        :param interleaving_size: Defines the interleaving size of the multi-
                                  channel memory system. By default, it is
                                  equivalent to the atom size, i.e., 64.
        """
        num_channels = _try_convert(num_channels, int)
        interleaving_size = _try_convert(interleaving_size, int)

        if size:
            size = _try_convert(size, str)

        if addr_mapping:
            addr_mapping = _try_convert(addr_mapping, str)

        super().__init__()
        self._dram_class = dram_interface_class
        self._num_channels = num_channels

        if not _isPow2(interleaving_size):
            raise ValueError("Memory interleaving size should be a power of 2")
        self._intlv_size = interleaving_size

        if addr_mapping:
            self._addr_mapping = addr_mapping
        else:
            self._addr_mapping = self._dram_class.addr_mapping.value

        if size:
            self._size = toMemorySize(size)
        else:
            self._size = self._get_dram_size(num_channels, self._dram_class)

        self._create_mem_interfaces_controller()

    def _create_mem_interfaces_controller(self):
        self._dram = [
            self._dram_class(addr_mapping=self._addr_mapping)
            for _ in range(self._num_channels)
        ]

        self.mem_ctrl = [
            MemCtrl(dram=self._dram[i]) for i in range(self._num_channels)
        ]

    def _get_dram_size(self, num_channels: int, dram: DRAMInterface) -> int:
        return num_channels * (
            dram.device_size.value
            * dram.devices_per_rank.value
            * dram.ranks_per_channel.value
        )

    def _interleave_addresses(self):
        if self._addr_mapping == "RoRaBaChCo":
            rowbuffer_size = (
                self._dram_class.device_rowbuffer_size.value
                * self._dram_class.devices_per_rank.value
            )
            intlv_low_bit = log(rowbuffer_size, 2)
        elif self._addr_mapping in ["RoRaBaCoCh", "RoCoRaBaCh"]:
            intlv_low_bit = log(self._intlv_size, 2)
        else:
            raise ValueError(
                "Only these address mappings are supported: "
                "RoRaBaChCo, RoRaBaCoCh, RoCoRaBaCh"
            )

        intlv_bits = log(self._num_channels, 2)
        for i, ctrl in enumerate(self.mem_ctrl):
            ctrl.dram.range = AddrRange(
                start=self._mem_range.start,
                size=self._mem_range.size(),
                intlvHighBit=intlv_low_bit + intlv_bits - 1,
                xorHighBit=0,
                intlvBits=intlv_bits,
                intlvMatch=i,
            )

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        if self._intlv_size < int(board.get_cache_line_size()):
            raise ValueError(
                "Memory interleaving size can not be smaller than"
                " board's cache line size.\nBoard's cache line size: "
                f"{board.get_cache_line_size()}\n, This memory's interleaving "
                f"size: {self._intlv_size}"
            )

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        return [(ctrl.dram.range, ctrl.port) for ctrl in self.mem_ctrl]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> List[MemCtrl]:
        return [ctrl for ctrl in self.mem_ctrl]

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        """Need to add support for non-contiguous non overlapping ranges in
        the future.
        """
        if len(ranges) != 1 or ranges[0].size() != self._size:
            raise Exception(
                "Multi channel memory controller requires a single range "
                "which matches the memory's size.\n"
                f"The range size: {range[0].size()}\n"
                f"This memory's size: {self._size}"
            )
        self._mem_range = ranges[0]
        self._interleave_addresses()
