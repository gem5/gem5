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

from typing import Type

from m5.objects.BaseO3CPU import BaseO3CPU
from m5.objects.BranchPredictor import BranchPredictor
from m5.proxy import Parent
from m5.util import inform

from .....utils.override import overrides
from ....modifier import Modifier
from .o3_cpu_modifier import O3CPUModifier


class O3BranchPredModifier(O3CPUModifier):
    """Class to implement modifying the branch predictor in an O3CPU.

    This class inherits from O3CPUModifier and implements the _do_modification
    method.

    parameters:
    branch_pred_cls: The class of the branch predictor to be set.
    **params: The parameters to be passed to the branch predictor.
    """

    def __init__(
        self, branch_pred_cls: Type[BranchPredictor], **params
    ) -> None:
        description = (
            f"Sets the branch predictor for an O3 CPU to {branch_pred_cls}."
        )
        super().__init__(description)
        self._branch_pred_cls = branch_pred_cls
        if "numThreads" not in params:
            inform(
                f"numThreads not passed as **params argument. "
                "Defaulting to the proxy param Parent.numThreads."
            )
            params["numThreads"] = Parent.numThreads
        self._params = params

    @overrides(Modifier)
    def _do_modification(self, sim_object: BaseO3CPU) -> None:
        sim_object.branchPred = self._branch_pred_cls(**self._params)
