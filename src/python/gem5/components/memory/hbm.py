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

""" HBM2 memory system using HBMCtrl
"""

from .memory import ChanneledMemory
from .abstract_memory_system import AbstractMemorySystem
from math import log
from ...utils.override import overrides
from m5.objects import AddrRange, DRAMInterface, HBMCtrl, Port
from typing import Type, Optional, Union, Sequence, Tuple
from .memory import _try_convert
from .dram_interfaces.hbm import HBM_2000_4H_1x64


class HighBandwidthMemory(ChanneledMemory):
    """
    This class extends ChanneledMemory and can be used to create HBM based
    memory system where a single physical channel contains two pseudo channels.
    This is supposed to be used with the HBMCtrl and two dram (HBM2) interfaces
    per channel.
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
            this memory controller
        :param num_channels: The number of channels that needs to be
        simulated
        :param size: Optionally specify the size of the DRAM controller's
            address space. By default, it starts at 0 and ends at the size of
            the DRAM device specified
        :param addr_mapping: Defines the address mapping scheme to be used.
            If None, it is defaulted to addr_mapping from dram_interface_class.
        :param interleaving_size: Defines the interleaving size of the multi-
            channel memory system. By default, it is equivalent to the atom
            size, i.e., 64.
        """
        super().__init__(
            dram_interface_class,
            num_channels,
            interleaving_size,
            size,
            addr_mapping,
        )

        _num_channels = _try_convert(num_channels, int)

    @overrides(ChanneledMemory)
    def _create_mem_interfaces_controller(self):
        self._dram = [
            self._dram_class(addr_mapping=self._addr_mapping)
            for _ in range(self._num_channels)
        ]
        self._dram_2 = [
            self._dram_class(addr_mapping=self._addr_mapping)
            for _ in range(self._num_channels)
        ]

        self.mem_ctrl = [
            HBMCtrl(
                dram=self._dram[i],
                dram_2=self._dram_2[i],
                disable_sanity_check=True,
            )
            for i in range(self._num_channels)
        ]

    @overrides(ChanneledMemory)
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
        mask_list = []

        for ib in range(int(intlv_bits)):
            mask_list.append(1 << int(ib + intlv_low_bit))

        # for interleaving across pseudo channels (at 64B currently)
        mask_list.insert(0, 1 << 6)
        for i, ctrl in enumerate(self.mem_ctrl):
            ctrl.dram.range = AddrRange(
                start=self._mem_range.start,
                size=self._mem_range.size(),
                masks=mask_list,
                intlvMatch=(i << 1) | 0,
            )
            ctrl.dram_2.range = AddrRange(
                start=self._mem_range.start,
                size=self._mem_range.size(),
                masks=mask_list,
                intlvMatch=(i << 1) | 1,
            )

    @overrides(ChanneledMemory)
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        intlv_bits = log(self._num_channels, 2)
        mask_list = []

        for ib in range(int(intlv_bits)):
            mask_list.append(1 << int(ib + log(self._intlv_size, 2)))
        addr_ranges = []
        for i in range(len(self.mem_ctrl)):
            addr_ranges.append(
                AddrRange(
                    start=self._mem_range.start,
                    size=self._mem_range.size(),
                    masks=mask_list,
                    intlvMatch=i,
                )
            )
        return [
            (addr_ranges[i], ctrl.port) for i, ctrl in enumerate(self.mem_ctrl)
        ]


def HBM2Stack(
    size: Optional[str] = "4GiB",
) -> AbstractMemorySystem:
    return HighBandwidthMemory(HBM_2000_4H_1x64, 8, 128, size=size)
