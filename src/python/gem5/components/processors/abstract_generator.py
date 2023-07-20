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

from abc import abstractmethod
from ...utils.override import overrides
from ..boards.mem_mode import MemMode
from .abstract_generator_core import AbstractGeneratorCore

from .abstract_processor import AbstractProcessor
from ..boards.abstract_board import AbstractBoard

from typing import List


def partition_range(
    min_addr: int, max_addr: int, num_partitions: int
) -> List[tuple]:
    assert (
        isinstance(min_addr, int)
        and isinstance(max_addr, int)
        and isinstance(num_partitions, int)
    )
    assert ((max_addr - min_addr) % num_partitions) == 0
    chunk_size = int((max_addr - min_addr) / num_partitions)
    return [
        (min_addr + chunk_size * i, min_addr + chunk_size * (i + 1))
        for i in range(num_partitions)
    ]


class AbstractGenerator(AbstractProcessor):
    """The abstract generator
    It defines the external interface of every generator component.
    """

    def __init__(self, cores: List[AbstractGeneratorCore]) -> None:
        """
        Create a list of AbstractGeneratorCore (which is an AbstractCore),
        to pass to the constructor of the AbstractProcessor. Due to the
        different prototypes for the constructor of different generator types
        inputs are noted as *args. This way the abstract method _create_cores
        could be called without AbstractGenerator having to know what the
        prototype for the constructor of the inheriting class is. It also
        limits the _create_cores function to only using positional arguments.
        keyword (optional arguments) are still allowable in the constructor of
        the inheriting classes.
        """
        super().__init__(cores=cores)

    @overrides(AbstractProcessor)
    def incorporate_processor(self, board: AbstractBoard) -> None:
        board.set_mem_mode(MemMode.TIMING)

    @abstractmethod
    def start_traffic(self) -> None:
        """
        Depending on what the internal generator core for inheriting classes is
        this method needs to be implemented in detail or implmeneted as pass.
        """
        raise NotImplementedError

    def _post_instantiate(self) -> None:
        self.start_traffic()
