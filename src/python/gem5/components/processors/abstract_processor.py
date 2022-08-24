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

from abc import ABCMeta, abstractmethod

from ...utils.requires import requires
from .abstract_core import AbstractCore

from m5.objects import SubSystem

from ..boards.abstract_board import AbstractBoard
from ...isas import ISA

from typing import List, Optional


class AbstractProcessor(SubSystem):
    __metaclass__ = ABCMeta

    def __init__(
        self,
        cores: Optional[List[AbstractCore]] = None,
        isa: ISA = ISA.NULL,
    ) -> None:
        """Set the cores on the processor
        Cores are optional for some processor types. If a processor does not
        set the cores here, it must override `get_num_cores` and `get_cores`
        """
        super().__init__()

        if cores:
            # In the stdlib we assume the system processor conforms to a single
            # ISA target.
            assert len(set(core.get_isa() for core in cores)) == 1
            self.cores = cores
            self._isa = cores[0].get_isa()
        else:
            self._isa = isa

    def get_num_cores(self) -> int:
        assert getattr(self, "cores")
        return len(self.cores)

    def get_cores(self) -> List[AbstractCore]:
        assert getattr(self, "cores")
        return self.cores

    def get_isa(self) -> ISA:
        return self._isa

    @abstractmethod
    def incorporate_processor(self, board: AbstractBoard) -> None:
        raise NotImplementedError
