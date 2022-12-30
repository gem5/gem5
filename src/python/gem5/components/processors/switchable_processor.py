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


from .simple_core import SimpleCore
from .abstract_core import AbstractCore
from .cpu_types import CPUTypes

import m5

from typing import Dict, List

from .abstract_processor import AbstractProcessor
from ..boards.abstract_board import AbstractBoard
from ...utils.override import *


class SwitchableProcessor(AbstractProcessor):
    """
    This class can be used to setup a switchable processor/processors on a
    system using SimpleCores.

    Though this class can be used directly, it is best inherited from. See
    "SimpleSwitchableCPU" for an example of this.
    """

    def __init__(
        self,
        switchable_cores: Dict[str, List[SimpleCore]],
        starting_cores: str,
    ) -> None:

        if starting_cores not in switchable_cores.keys():
            raise AssertionError(
                f"Key {starting_cores} cannot be found in the "
                "switchable_processors dictionary."
            )

        self._current_cores = switchable_cores[starting_cores]
        self._switchable_cores = switchable_cores

        # In the stdlib we assume the system processor conforms to a single
        # ISA target.
        assert len(set(core.get_isa() for core in self._current_cores)) == 1
        super().__init__(isa=self._current_cores[0].get_isa())

        for name, core_list in self._switchable_cores.items():
            # Use the names from the user as the member variables
            # This makes the stats print more nicely.
            setattr(self, name, core_list)
            for core in core_list:
                core.set_switched_out(core not in self._current_cores)

        self._prepare_kvm = any(
            core.is_kvm_core() for core in self._all_cores()
        )

        if self._prepare_kvm:
            from m5.objects import KvmVM

            self.kvm_vm = KvmVM()

    @overrides(AbstractProcessor)
    def incorporate_processor(self, board: AbstractBoard) -> None:

        # This is a bit of a hack. The `m5.switchCpus` function, used in the
        # "switch_to_processor" function, requires the System simobject as an
        # argument. We therefore need to store the board when incorporating the
        # procsesor
        self._board = board

        if self._prepare_kvm:
            # To get the KVM CPUs to run on different host CPUs
            # Specify a different event queue for each CPU
            kvm_cores = [
                core for core in self._all_cores() if core.is_kvm_core()
            ]
            for i, core in enumerate(kvm_cores):
                for obj in core.get_simobject().descendants():
                    obj.eventq_index = 0
                core.get_simobject().eventq_index = i + 1

    @overrides(AbstractProcessor)
    def get_num_cores(self) -> int:
        # Note: This is a special case where the total number of cores in the
        # design is not the number of cores, due to some being switched out.
        return len(self._current_cores)

    @overrides(AbstractProcessor)
    def get_cores(self) -> List[AbstractCore]:
        return self._current_cores

    def _all_cores(self):
        for core_list in self._switchable_cores.values():
            for core in core_list:
                yield core

    def switch_to_processor(self, switchable_core_key: str):

        # Run various checks.
        if not hasattr(self, "_board"):
            raise AssertionError("The processor has not been incorporated.")

        if switchable_core_key not in self._switchable_cores.keys():
            raise AssertionError(
                f"Key {switchable_core_key} is not a key in the"
                " switchable_processor dictionary."
            )

        # Select the correct processor to switch to.
        to_switch = self._switchable_cores[switchable_core_key]

        # Run more checks.
        if to_switch == self._current_cores:
            raise AssertionError(
                "Cannot swap current cores with the current cores"
            )

        if len(to_switch) != len(self._current_cores):
            raise AssertionError(
                "The number of cores to swap in is not the same as the number "
                "already swapped in. This is not allowed."
            )

        current_core_simobj = [
            core.get_simobject() for core in self._current_cores
        ]
        to_switch_simobj = [core.get_simobject() for core in to_switch]

        # Switch the CPUs
        m5.switchCpus(
            self._board, list(zip(current_core_simobj, to_switch_simobj))
        )

        # Ensure the current processor is updated.
        self._current_cores = to_switch
