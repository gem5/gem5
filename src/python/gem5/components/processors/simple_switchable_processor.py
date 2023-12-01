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

from typing import Optional

from m5.util import warn

from ...isas import ISA
from ...utils.override import *
from ..boards.abstract_board import AbstractBoard
from ..boards.mem_mode import MemMode
from ..processors.cpu_types import (
    CPUTypes,
    get_mem_mode,
)
from ..processors.simple_core import SimpleCore
from .switchable_processor import SwitchableProcessor


class SimpleSwitchableProcessor(SwitchableProcessor):
    """
    A Simplified implementation of SwitchableProcessor where there is one
    processor at the start of the simuation, and another that can be switched
    to via the "switch" function later in the simulation. This is good for
    fast/detailed CPU setups.
    """

    def __init__(
        self,
        starting_core_type: CPUTypes,
        switch_core_type: CPUTypes,
        num_cores: int,
        isa: Optional[ISA] = None,
    ) -> None:
        """
        :param starting_core_type: The CPU type for each type in the processor
                                   to start with (i.e., when the simulation has
                                   just started).

        :param switch_core_types: The CPU type for each core, to be switched to.

        :param isa: The ISA of the processor. This argument is optional. If not
                    set the ``runtime.get_runtime_isa`` is used to determine the
                    ISA at runtime. **WARNING**: This functionality is deprecated.
                    It is recommended you explicitly set your ISA via
                    SimpleSwitchableProcessor construction.
        """

        if not isa:
            warn(
                "An ISA for the SimpleSwitchableProcessor was not set. This "
                "will result in usage of `runtime.get_runtime_isa` to obtain "
                "the ISA. This function is deprecated and will be removed in "
                "future releases of gem5. Please explicitly state the ISA "
                "via the processor constructor."
            )

        if num_cores <= 0:
            raise AssertionError("Number of cores must be a positive integer!")

        self._start_key = "start"
        self._switch_key = "switch"
        self._current_is_start = True

        self._mem_mode = get_mem_mode(starting_core_type)

        switchable_cores = {
            self._start_key: [
                SimpleCore(cpu_type=starting_core_type, core_id=i, isa=isa)
                for i in range(num_cores)
            ],
            self._switch_key: [
                SimpleCore(cpu_type=switch_core_type, core_id=i, isa=isa)
                for i in range(num_cores)
            ],
        }

        super().__init__(
            switchable_cores=switchable_cores, starting_cores=self._start_key
        )

    @overrides(SwitchableProcessor)
    def incorporate_processor(self, board: AbstractBoard) -> None:
        super().incorporate_processor(board=board)

        if (
            board.get_cache_hierarchy().is_ruby()
            and self._mem_mode == MemMode.ATOMIC
        ):
            warn(
                "Using an atomic core with Ruby will result in "
                "'atomic_noncaching' memory mode. This will skip caching "
                "completely."
            )
            self._mem_mode = MemMode.ATOMIC_NONCACHING
        board.set_mem_mode(self._mem_mode)

    def switch(self):
        """Switches to the "switched out" cores."""
        if self._current_is_start:
            self.switch_to_processor(self._switch_key)
        else:
            self.switch_to_processor(self._start_key)

        self._current_is_start = not self._current_is_start
