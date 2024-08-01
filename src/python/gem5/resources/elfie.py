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

from typing import (
    TYPE_CHECKING,
    List,
    Optional,
)

from m5.objects.PcCountTracker import PcCountTrackerManager
from m5.params import PcCountPair

if TYPE_CHECKING:
    from gem5.components.processors.abstract_processor import AbstractProcessor


class ELFieInfo:
    """Stores information to load/run ELFies.

    See https://github.com/intel/pinball2elf for more information.
    """

    def __init__(
        self,
        start_pc: Optional[str] = None,
        end_pc: Optional[str] = None,
        start_pc_count: Optional[str] = None,
        end_pc_count: Optional[str] = None,
        start: Optional["PcCountPair"] = None,
        end: Optional["PcCountPair"] = None,
        **kwargs
    ):
        self._start = start
        self._end = end

        if self._start is None:
            if start_pc is None or start_pc_count is None:
                raise ValueError(
                    "start_pc and start_pc_count must be provided"
                )
            self._start = PcCountPair(int(start_pc, 16), int(start_pc_count))

        if self._end is None:
            if end_pc is None or end_pc_count is None:
                raise ValueError("end_pc and end_pc_count must be provided")
            self._end = PcCountPair(int(end_pc, 16), int(end_pc_count))

        self._manager = PcCountTrackerManager()
        self._manager.targets = self.get_targets()

    def setup_processor(
        self,
        processor: "AbstractProcessor",
    ) -> None:
        """
        A function is used to setup a PC tracker in all the cores and
        connect all the tracker to the PC tracker manager to perform
        multithread PC tracking.

        :param processor: The processor used in the simulation configuration.
        """
        for core in processor.get_cores():
            core.add_pc_tracker_probe(self.get_targets(), self.get_manager())

    def get_targets(self) -> List[PcCountPair]:
        """Returns the complete list of targets PcCountPairs. That is, the
        PcCountPairs each region starts with as well as the relevant warmup
        intervals."""
        assert self._start and self._end
        return [self._start, self._end]

    def get_manager(self) -> PcCountTrackerManager:
        """Returns the PcCountTrackerManager for this ELFie data
        structure."""
        return self._manager
