# Copyright (c) 2023 The Regents of the University of California
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

from typing import Iterator

from m5.objects import BaseTrafficGen  # type: ignore
from m5.objects import PyTrafficGen  # type: ignore
from m5.params import Port
from m5.ticks import fromSeconds
from m5.util.convert import (
    toLatency,
    toMemoryBandwidth,
)

from gem5.components.processors.abstract_core import AbstractCore
from gem5.components.processors.abstract_generator_core import (
    AbstractGeneratorCore,
)
from gem5.utils.override import overrides


class StridedGeneratorCore(AbstractGeneratorCore):
    def __init__(
        self,
        duration: str,
        rate: str,
        block_size: int,
        superblock_size: int,
        stride_size: int,
        min_addr: int,
        max_addr: int,
        offset: int,
        rd_perc: int,
        data_limit: int,
    ) -> None:
        super().__init__()

        self.generator = PyTrafficGen()
        self._duration = duration
        self._rate = rate
        self._block_size = block_size
        self._superblock_size = superblock_size
        self._stride_size = stride_size
        self._min_addr = min_addr
        self._max_addr = max_addr
        self._offset = offset
        self._rd_perc = rd_perc
        self._data_limit = data_limit

    @overrides(AbstractCore)
    def connect_dcache(self, port: Port) -> None:
        self.generator.port = port

    def _set_traffic(self) -> None:
        self._traffic = self._create_traffic()

    def _create_traffic(self) -> Iterator[BaseTrafficGen]:
        duration = fromSeconds(toLatency(self._duration))
        rate = toMemoryBandwidth(self._rate)
        period = fromSeconds(self._block_size / rate)
        min_period = period
        max_period = period
        yield self.generator.createStrided(
            duration,
            self._min_addr,
            self._max_addr,
            self._offset,
            self._block_size,
            self._superblock_size,
            self._stride_size,
            min_period,
            max_period,
            self._rd_perc,
            self._data_limit,
        )
        yield self.generator.createExit(0)

    @overrides(AbstractGeneratorCore)
    def start_traffic(self) -> None:
        self._set_traffic()
        self.generator.start(self._traffic)
