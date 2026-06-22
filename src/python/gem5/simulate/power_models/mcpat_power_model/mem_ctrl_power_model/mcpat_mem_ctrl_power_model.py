# Copyright (c) 2026, University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.objects import (
    MemCtrl,
    PowerModel,
    PowerModelPyFunc,
)

from ..base_mcpat_power_model import ActEnergyType
from .mcpat_mem_backend_power_model import McPATMemBackendPowerModel
from .mcpat_mem_frontend_power_model import McPATMemFrontendPowerModel
from .mcpat_mem_phy_power_model import McPATMemPhyPowerModel


class McPATMemCtrlPowerOn(PowerModelPyFunc):
    """
    This class models the Memory Controller using McPAT's power model.
    This has three main components: The frontend, the backend, and the
    physical layer. Each of these are in their own separate class

    It is assumed that when you are grabbing power for all of these
    components, they are converted into watts using the processor's clock
    (i.e., P = total_mem_energy / workload_exec_time)
    """

    def __init__(
        self,
        mem_ctrl: MemCtrl,
        act_energies: ActEnergyType,
        print_output: bool = False,
    ):
        super().__init__()
        self._print_output = print_output
        self._phy = McPATMemPhyPowerModel(
            mem_ctrl=mem_ctrl, act_energies=act_energies
        )
        self._frontend = McPATMemFrontendPowerModel(
            mem_ctrl=mem_ctrl, act_energies=act_energies
        )
        self._backend = McPATMemBackendPowerModel(
            mem_ctrl=mem_ctrl, act_energies=act_energies
        )

        self.dyn = self.dynamic_power
        self.st = self.static_power

    def static_power(self):
        # Placeholder value for static power.
        return 1.0

    def dynamic_power(self):
        total = self._backend.dynamic_power()
        total += self._frontend.dynamic_power()
        total += self._phy.dynamic_power()
        if self._print_output:
            self.print_mcpat(6, total)
        return total

    def print_mcpat(self, indent, total):
        print("*" * 80)
        print("Memory Controller:")
        print(" " * indent + f"Runtime Dynamic = {total}\n")
        self._frontend.print_mcpat(indent)
        self._backend.print_mcpat(indent)
        self._phy.print_mcpat(indent)
        print("*" * 80)


class McPATMemCtrlPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class McPATMemCtrlPowerModel(PowerModel):
    def __init__(self, mem_ctrl: MemCtrl, act_energies: ActEnergyType):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            McPATMemCtrlPowerOn(
                mem_ctrl=mem_ctrl, act_energies=act_energies
            ),  # ON
            McPATMemCtrlPowerOff(),  # CLK_GATED
            McPATMemCtrlPowerOff(),  # SRAM_RETENTION
            McPATMemCtrlPowerOff(),  # OFF
        ]
