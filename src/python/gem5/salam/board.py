# Copyright (c) 2026 Akanksha Chaudhari
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


# A bare-metal ARM board with gem5-SALAM accelerator clusters.

from m5.objects import (
    ArmDefaultRelease,
    ArmFsWorkload,
    ArmRelease,
    VExpress_GEM5_Base,
    VExpress_GEM5_V1,
)

from ..components.boards.abstract_board import AbstractBoard
from ..components.boards.arm_board import ArmBoard
from ..components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from ..components.cachehierarchies.classic.abstract_classic_cache_hierarchy import (
    AbstractClassicCacheHierarchy,
)
from ..components.memory.abstract_memory_system import AbstractMemorySystem
from ..components.processors.abstract_processor import AbstractProcessor
from ..utils.override import overrides
from .cluster import build_clusters


class SALAMArmBoard(ArmBoard):
    """Bare-metal ARM board using a classic cache hierarchy with a membus."""

    def __init__(
        self,
        clk_freq: str,
        processor: AbstractProcessor,
        memory: AbstractMemorySystem,
        cache_hierarchy: AbstractCacheHierarchy,
        platform: VExpress_GEM5_Base = VExpress_GEM5_V1(),
        release: ArmRelease = ArmDefaultRelease(),
    ) -> None:
        if not isinstance(
            cache_hierarchy, AbstractClassicCacheHierarchy
        ) or not hasattr(cache_hierarchy, "membus"):
            raise ValueError(
                "SALAMArmBoard needs a classic cache hierarchy with a "
                f"membus, not {type(cache_hierarchy).__name__}"
            )
        super().__init__(
            clk_freq=clk_freq,
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
            platform=platform,
            release=release,
        )

    @overrides(ArmBoard)
    def _setup_board(self) -> None:
        super()._setup_board()
        self.workload = ArmFsWorkload(dtb_addr=0)

    @overrides(ArmBoard)
    def _setup_io_devices(self) -> None:
        super()._setup_io_devices()
        self.realview.uart[0].end_on_eot = True

    def set_bare_metal_binary(self, path: str) -> None:
        """Set the ARM ELF before attaching accelerators."""
        self._set_fullsystem(True)
        self.workload.object_file = path
        self.set_is_workload_set(True)

    def attach_accelerators(
        self, working_dir: str, config_name: str = "config.yml"
    ):
        if not self.is_workload_set():
            raise RuntimeError(
                "set_bare_metal_binary must be called before "
                "attach_accelerators"
            )
        return build_clusters(
            self,
            working_dir,
            config_name,
            membus=self.get_cache_hierarchy().membus,
        )

    @overrides(ArmBoard)
    def _pre_instantiate(self, full_system=None):
        # Bare-metal execution does not need ArmBoard's Linux boot setup.
        return AbstractBoard._pre_instantiate(self, full_system=full_system)
