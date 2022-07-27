# Copyright (c) 2022 Fraunhofer IESE
# All rights reserved
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

import m5
import os
import configparser

from m5.objects import DRAMSys, AddrRange, Port, MemCtrl, Gem5ToTlmBridge32
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem

from typing import Optional, Tuple, Sequence, List


class DRAMSysMem(AbstractMemorySystem):
    def __init__(
        self,
        configuration: str,
        size: str,
        resource_directory: str,
        recordable: bool,
    ) -> None:
        """
        :param configuration: Path to the base configuration JSON for DRAMSys.
        :param size: Memory size of DRAMSys. Must match the size specified in JSON configuration.
        :param resource_directory: Path to the base resource directory for DRAMSys.
        :param recordable: Whether the database recording feature of DRAMSys is enabled.
        """
        super().__init__()
        self.dramsys = DRAMSys(
            configuration=configuration,
            resource_directory=resource_directory,
            recordable=recordable,
        )

        self._size = toMemorySize(size)
        self._bridge = Gem5ToTlmBridge32()
        self.dramsys.port = self._bridge.tlm

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        pass

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        return [(self.dramsys.range, self._bridge.gem5)]

    @overrides(AbstractMemorySystem)
    def get_memory_controllers(self) -> List[MemCtrl]:
        return [self.dramsys]

    @overrides(AbstractMemorySystem)
    def get_size(self) -> int:
        return self._size

    @overrides(AbstractMemorySystem)
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        if len(ranges) != 1 or ranges[0].size() != self._size:
            raise Exception(
                "DRAMSys memory controller requires a single "
                "range which matches the memory's size."
            )
        self.dramsys.range = ranges[0]
