# Copyright (c) 2025 Technical University of Munich
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

from m5.objects import (
    LTAGE,
    BranchPredictor,
    SimpleBTB,
)
from m5.util import warn

from ...isas import ISA
from ..processors.simple_core import SimpleCore
from .base_cpu_processor import BaseCPUProcessor
from .cpu_types import CPUTypes


class DecoupledProcessor(BaseCPUProcessor):
    """
    A DecoupledProcessor contains a number of O3 cores configured with
    a decoupled frontend.
    By default each core is equiped with a 8K Entry BTB and a 32K LTAGE
    branch predictor.
    """

    def __init__(
        self, num_cores: int, isa: ISA, decoupled: bool = True
    ) -> None:
        """
        :param num_cores: The number of CPU cores in the processor.

        :param isa: The ISA of the processor.

        :param decouple: Enable/Disable the decoupled frontend.
        """
        super().__init__(
            cores=[
                SimpleCore(cpu_type=CPUTypes.O3, core_id=i, isa=isa)
                for i in range(num_cores)
            ]
        )

        for c in self.get_cores():
            # First the fetch buffer and fetch target size. We want double the
            # size of the fetch buffer to be able to run ahead of fetch
            c.core.fetchBufferSize = 16
            c.core.fetchTargetWidth = 32

            # The decoupled front-end leverages the BTB to find branches in the
            # fetch stream. Starting from the end of the last fetch target it
            # will search all addresses until a hit. However, for fixed size
            # ISA's like Arm only every n-th address must be checked.
            if isa == ISA.ARM:
                c.core.minInstSize = (
                    4  # Note Arm has a 2 byte thumb mode but we ignore it here
                )
            elif isa == ISA.RISCV:
                c.core.minInstSize = 2  # RiscV has a 2 byte compressed mode.
            else:  # Variable length ISA (x86) must search every byte
                c.core.minInstSize = 1

            # The `decoupledFrontEnd` parameter enables the decoupled
            # front-end. Disable it to get the baseline.
            if decoupled:
                c.core.decoupledFrontEnd = True

            # Setup branch predictor
            c.core.branchPred = BranchPredictor(
                # For fixed size ISAs like Arm and RISC-V the lower order bits
                # are always zero and do not add any entropy to the branch
                # predictor. The `instShiftAmt` is used to shift the address by
                # the number of unused bits.
                instShiftAmt=(
                    2 if isa == ISA.ARM else 1 if isa == ISA.RISCV else 0
                ),
                btb=SimpleBTB(
                    numEntries=8 * 1024,
                    associativity=8,
                ),
                conditionalBranchPred=LTAGE(),
                requiresBTBHit=True,
                # Decoupled frontend requires a taken-only global history
                takenOnlyHistory=True,
            )
