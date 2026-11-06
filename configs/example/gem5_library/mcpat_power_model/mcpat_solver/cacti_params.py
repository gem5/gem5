from .cacti_tech_params import CactiTechParams
from .cacti_wire_params import CactiWireParams
from .config_input_params import McPATMachineConfig


class CactiParams(CactiTechParams, CactiWireParams):
    def __init__(self, machine_config):
        self._machine_config = machine_config
        CactiTechParams.__init__(self, machine_config)
        CactiWireParams.__init__(self, machine_config)
        self.init_tech_params()
        self.init_wire_params()
        self._wp = self._wire_params
        self._tp = self._tech_params

    def get_pmos_to_nmos_sz_ratio(self):
        return self._tech_params["n_to_p_eff_curr_drv_ratio"]
