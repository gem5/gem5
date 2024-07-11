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

from __future__ import annotations

from abc import abstractmethod

from m5.objects.SimObject import SimObject


class Modifier:
    """Abstract class that can be used to modify SimObjects on a board.

    This class allows for fast and reusable modifications to SimObjects on a
    board. It will be useful when searching parameter space for SimObjects used
    by components in the standard library without having to extend a component
    class to exposes that parameter to the user through the constructor.

    Each modifier needs to implement the following methods:
    - _get_simobjects_from_board: This method should return a list of SimObjects
    (not components, e.g. O3CPU and not SimpleProcessor) that it will modify.
    - _do_modification: This method should actually do the modification it is
    supposed to do. E.g. it changes the BranchPredictor of a cpu core.

    For an example implementation of a Modifier based class look at:
        src/python/gem5/components/processors/modifiers/o3_modifiers/o3_cpu_modifier.py
        src/python/gem5/components/processors/modifiers/o3_modifiers/branchpred_modifier.py
    """

    def __init__(self, description: str) -> None:
        self._desc = description

    def get_desc(self):
        return self._desc

    def __str__(self):
        return self.get_desc()

    @abstractmethod
    def _get_simobjects_from_board(
        self, board: AbstractBoard
    ) -> list[SimObject]:
        raise NotImplementedError

    @abstractmethod
    def _do_modification(self, sim_object: SimObject) -> None:
        raise NotImplementedError

    def apply(self, board: AbstractBoard) -> None:
        for sim_object in self._get_simobjects_from_board(board):
            self._do_modification(sim_object)
