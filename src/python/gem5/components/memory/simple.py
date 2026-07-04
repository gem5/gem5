# Copyright (c) 2025 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
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

"""Simple memory controllers"""

from math import log
from typing import (
    List,
    Tuple,
    Union,
)
from collections.abc import Sequence

from m5.objects import (
    AbstractMemory,
    MemCtrl,
    SimpleMemory,
)
from m5.params import (
    AddrRange,
    Port,
)
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem


def _isPow2(num):
    log_num = int(log(num, 2))
    return 2**log_num == num


class SingleChannelSimpleMemory(AbstractMemorySystem):
    """A class to implement single channel memory system using SimpleMemory

    This class takes latency, latency variation, and bandwidth and configures
    a memory with those values. It could be used for studies that do not target
    memory subsystem design.
    """

    def __init__(
        self, latency: str, latency_var: str, bandwidth: str, size: str
    ):
        """
        :param latency: The average of request to response latency.
        :param latency_var: The variance of request to response latency.
        :param bandwidth: Combined read and write bandwidth.
        :param size: Size of the memory.
        """
        super().__init__()

        self.module = SimpleMemory(
            latency=latency,
            latency_dist_param=latency_var,
            bandwidth=bandwidth,
        )
        self._size = toMemorySize(size)

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        pass

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[tuple[AddrRange, Port]]:
        return [(self.module.range, self.module.port)]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> list[MemCtrl]:
        return [self.module]

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def get_uninterleaved_range(self) -> list[AddrRange]:
        return [self.module.range]

    @overrides(AbstractMemorySystem)
    def get_mem_interfaces(self) -> list[AbstractMemory]:
        return [self.module]

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: list[AddrRange]) -> None:
        if len(ranges) != 1 or ranges[0].size() != self._size:
            raise Exception(
                "Simple single channel memory controller requires a single "
                "range which matches the memory's size."
            )
        self.module.range = ranges[0]


class MultiChannelSimpleMemory(AbstractMemorySystem):
    """A class to implement a multi-channel memory system using SimpleMemory.

    Each channel is a SimpleMemory SimObject with identical latency and
    bandwidth parameters. Addresses are interleaved across channels at a
    configurable granularity (default: 64 B, i.e., one cache line per
    channel at a time) using gem5's AddrRange bit-mask interleaving.

    This is useful for bandwidth-scaling experiments where the modelled
    memory technology is not important, but the aggregate bandwidth is.
    The number of channels must be a power of 2.
    """

    def __init__(
        self,
        num_channels: int | str,
        latency: str,
        latency_var: str,
        bandwidth: str,
        size: str,
        interleaving_size: int | str = 64,
    ):
        """
        :param num_channels: Number of SimpleMemory channels (must be a
                             power of 2).
        :param latency: Average request-to-response latency per channel.
        :param latency_var: Variance of request-to-response latency.
        :param bandwidth: Combined read/write bandwidth *per channel*.
                          Total aggregate bandwidth scales linearly with the
                          number of channels.
        :param size: Total size of the memory system across all channels.
        :param interleaving_size: Interleaving granularity in bytes.
                                  Must be a power of 2 and >= the board's
                                  cache line size. Defaults to 64 B.
        """
        num_channels = int(num_channels)
        interleaving_size = int(interleaving_size)

        if not _isPow2(num_channels):
            raise ValueError(
                "Number of SimpleMemory channels must be a power of 2."
            )
        if not _isPow2(interleaving_size):
            raise ValueError("Memory interleaving size must be a power of 2.")

        super().__init__()
        self._num_channels = num_channels
        self._intlv_size = interleaving_size
        self._size = toMemorySize(size)

        self.mem_ctrl = [
            SimpleMemory(
                latency=latency,
                latency_var=latency_var,
                bandwidth=bandwidth,
            )
            for _ in range(num_channels)
        ]

    def _interleave_addresses(self) -> None:
        intlv_bits = int(log(self._num_channels, 2))
        intlv_low_bit = int(log(self._intlv_size, 2))
        for i, module in enumerate(self.mem_ctrl):
            module.range = AddrRange(
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
                "Memory interleaving size cannot be smaller than the board's "
                f"cache line size.\nBoard cache line size: "
                f"{board.get_cache_line_size()}\nThis memory's interleaving "
                f"size: {self._intlv_size}"
            )

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[tuple[AddrRange, Port]]:
        return [(m.range, m.port) for m in self.mem_ctrl]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> list[MemCtrl]:
        return list(self.mem_ctrl)

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: list[AddrRange]) -> None:
        if len(ranges) != 1 or ranges[0].size() != self._size:
            raise Exception(
                "Multi-channel SimpleMemory requires a single range which "
                "matches the memory's total size.\n"
                f"Range size: {ranges[0].size()}\n"
                f"Memory size: {self._size}"
            )
        self._mem_range = ranges[0]
        self._interleave_addresses()

    @overrides(AbstractMemorySystem)
    def get_uninterleaved_range(self) -> list[AddrRange]:
        return [self._mem_range]

    @overrides(AbstractMemorySystem)
    def get_mem_interfaces(self) -> list[AbstractMemory]:
        return list(self.mem_ctrl)
