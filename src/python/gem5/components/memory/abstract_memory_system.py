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

from abc import (
    ABCMeta,
    abstractmethod,
)
from typing import (
    List,
    Sequence,
    Tuple,
)

from m5.objects import (
    AddrRange,
    MemCtrl,
    Port,
    SubSystem,
)

from ..boards.abstract_board import AbstractBoard


class AbstractMemorySystem(SubSystem):
    __metaclass__ = ABCMeta

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def incorporate_memory(self, board: AbstractBoard) -> None:
        """This function completes all of the necessary steps to add this
        memory system to the board."""
        raise NotImplementedError

    @abstractmethod
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        """Get the ports to connect this memory system to the cache."""
        raise NotImplementedError

    @abstractmethod
    def get_memory_controllers(self) -> List[MemCtrl]:
        """Get all of the memory controllers in this memory system."""
        raise NotImplementedError

    @abstractmethod
    def get_size(self) -> int:
        """Returns the total size of the memory system."""
        raise NotImplementedError

    @abstractmethod
    def set_memory_range(self, ranges: List[AddrRange]) -> None:
        """Set the total range for this memory system.

        May pass multiple non-overlapping ranges. The total size of the ranges
        should match the size of the memory.

        If this memory system is incompatible with the ranges, an exception
        will be raised.
        """
        raise NotImplementedError

    def _post_instantiate(self) -> None:
        """Called to set up anything needed after ``m5.instantiate``."""
        pass
