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

from pathlib import Path
from typing import (
    List,
    Optional,
    Sequence,
    Tuple,
)

from m5.objects import (
    AddrRange,
    DRAMSys,
    Gem5ToTlmBridge32,
    MemCtrl,
    Port,
    SystemC_Kernel,
)
from m5.util.convert import toMemorySize

from ...utils.override import overrides
from ..boards.abstract_board import AbstractBoard
from .abstract_memory_system import AbstractMemorySystem

DEFAULT_DRAMSYS_DIRECTORY = Path("ext/dramsys/DRAMSys")


class DRAMSysMem(AbstractMemorySystem):
    """
    A DRAMSys memory controller.

    This class requires gem5 to be built with DRAMSys (see ext/dramsys).
    The specified memory size does not control the simulated memory size but it's sole purpose is
    to notify gem5 of DRAMSys's memory size.
    Therefore it has to match the DRAMSys configuration.
    DRAMSys is configured using JSON files, whose base configuration has to be passed as a
    parameter. Sub-configs are specified relative to the optional resource directory parameter.
    """

    def __init__(
        self,
        configuration: str,
        size: str,
        resource_directory: Optional[str] = None,
    ) -> None:
        """
        :param configuration: Path to the base configuration JSON for DRAMSys.
        :param size: Memory size of DRAMSys. Must match the size specified in JSON configuration.
        :param resource_directory: Path to the base resource directory for DRAMSys.
        """
        super().__init__()

        resource_directory_path = (
            DEFAULT_DRAMSYS_DIRECTORY / "configs"
            if resource_directory is None
            else Path(resource_directory)
        )

        self.dramsys = DRAMSys(
            configuration=configuration,
            resource_directory=resource_directory_path.as_posix(),
        )

        self._size = toMemorySize(size)
        self.bridge = Gem5ToTlmBridge32()
        self.dramsys.tlm = self.bridge.tlm
        self.kernel = SystemC_Kernel()

    @overrides(AbstractMemorySystem)
    def incorporate_memory(self, board: AbstractBoard) -> None:
        pass

    @overrides(AbstractMemorySystem)
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        return [(self.dramsys.range, self.bridge.gem5)]

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
        self.bridge.addr_ranges = ranges[0]


class DRAMSysDDR4_1866(DRAMSysMem):
    """
    An example DDR4 1866 DRAMSys configuration.
    """

    def __init__(self):
        super().__init__(
            configuration=(
                DEFAULT_DRAMSYS_DIRECTORY / "configs/ddr4-example.json"
            ).as_posix(),
            size="4GiB",
        )


class DRAMSysDDR3_1600(DRAMSysMem):
    """
    An example DDR3 1600 DRAMSys configuration.
    """

    def __init__(self):
        super().__init__(
            configuration=(
                DEFAULT_DRAMSYS_DIRECTORY / "configs/ddr3-gem5-se.json"
            ).as_posix(),
            size="1GiB",
        )


class DRAMSysLPDDR4_3200(DRAMSysMem):
    """
    An example LPDDR4 3200 DRAMSys configuration.
    """

    def __init__(self):
        super().__init__(
            configuration=(
                DEFAULT_DRAMSYS_DIRECTORY / "configs/lpddr4-example.json"
            ).as_posix(),
            size="1GiB",
        )


class DRAMSysHBM2(DRAMSysMem):
    """
    An example HBM2 DRAMSys configuration.
    """

    def __init__(self):
        super().__init__(
            configuration=(
                DEFAULT_DRAMSYS_DIRECTORY / "configs/hbm2-example.json"
            ).as_posix(),
            size="1GiB",
        )
