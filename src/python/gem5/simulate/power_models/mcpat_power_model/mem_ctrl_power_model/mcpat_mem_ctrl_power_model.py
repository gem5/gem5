from m5.objects import (
    MemCtrl,
    PowerModel,
    PowerModelPyFunc,
)

from .mcpat_mem_backend_power_model import McPATMemBackendPowerModel
from .mcpat_mem_frontend_power_model import McPATMemFrontendPowerModel
from .mcpat_mem_phy_power_model import McPATMemPhyPowerModel


class McPATMemCtrlPowerOn(PowerModelPyFunc):
    def __init__(self, mem_ctrl: MemCtrl, act_energies):
        """mem must be a MemCtrl!"""
        super().__init__()
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
        return 1.0

    def dynamic_power(self):
        total = self._backend.dynamic_power()
        total += self._frontend.dynamic_power()
        total += self._phy.dynamic_power()
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
    def __init__(self, mem_ctrl, act_energies):
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
