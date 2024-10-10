# Copyright (c) 2024 The Regents of the University of California
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

from typing import List

from m5.objects.BaseO3CPU import BaseO3CPU

from .....utils.override import overrides
from ....boards.abstract_board import AbstractBoard
from ....modifier import Modifier


class O3CPUModifier(Modifier):
    """Class that enables modifying objects of BaseO3CPU.
    This class implements the _get_simobjects_from_board method to get the
    cores from the board and check if they are of type BaseO3CPU.

    Other modifiers can inherit from this to modify BaseO3CPU objects.

    parameters:
    description: A string describing the modification to be made.
    """

    def __init__(self, description: str) -> None:
        super().__init__(description)

    @overrides(Modifier)
    def _get_simobjects_from_board(
        self, board: AbstractBoard
    ) -> List[BaseO3CPU]:
        cores = [
            core.get_simobject() for core in board.get_processor().get_cores()
        ]
        for core in cores:
            if not isinstance(core, BaseO3CPU):
                raise ValueError(
                    f"Expected core to be of type BaseO3CPU, got {type(core)}"
                )
        return cores
