from copy import deepcopy
from dataclasses import dataclass
from math import (
    ceil,
    log2,
)


class InputConfig:
    def __init__(self, params):
        self._config_params = deepcopy(params)
        self._defined_config_params = set()

    def validate(self):
        if set(self._config_params) != self._defined_config_params:
            error_str = (
                "Warning! Passed in dictionary has ",
                f"{set(self._config_params).difference(self._defined_config_params)}",
                "elements which are not in the parameters we accept!",
                f"McPAT Core Parameter List: {set(self._defined_config_params)}",
            )
            raise KeyError(error_str)

    def comma_sep_str_to_list(self, string):
        values = string.split(",")
        return list(map(int, values))


class McPATCoreConfig(InputConfig):
    def __init__(self, params):
        super().__init__(params)
        self._defined_config_params = {
            "clock_rate",
            "inst_len",
            "opcode_width",
            "isX86",
            "micro_opcode_width",
            "machine_type",
            "num_threads",
            "fetch_width",
            "num_ifetch_ports",
            "decode_width",
            "issue_width",
            "peak_issue_width",
            "commit_width",
            "fp_issue_width",
            "prediction_width",
            "pipelines_per_core",
            "pipeline_depth",
            "alus_per_core",
            "muls_per_core",
            "fpu_per_core",
            "inst_buffer_size",
            "dec_stream_buffer_sz",
            "inst_window_scheme",
            "inst_window_size",
            "fp_inst_window_size",
            "rob_size",
            "archi_regs_irf_size",
            "archi_regs_frf_size",
            "phys_regs_irf_size",
            "phys_regs_frf_size",
            "rename_scheme",
            "chkpt_depth",
            "reg_window_size",
            "lsu_order",
            "store_buffer_sz",
            "load_buffer_sz",
            "mem_ports",
            "ras_sz",
        }
        self.reconfigure_params()

    def reconfigure_params(self):
        self._config_params["arch_ireg_width"] = int(
            ceil(log2(self._config_params["archi_regs_irf_size"]))
        )
        self._config_params["arch_freg_width"] = int(
            ceil(log2(self._config_params["archi_regs_frf_size"]))
        )

        if self._config_params["machine_type"] == 0:
            if self._config_params["inst_window_scheme"] == 0:
                self._config_params["phy_ireg_width"] = int(
                    ceil(log2(self._config_params["phys_regs_irf_size"]))
                )
                self._config_params["phy_freg_width"] = int(
                    ceil(log2(self._config_params["phys_regs_frf_size"]))
                )


class McPATMachineConfig(InputConfig):
    def __init__(
        self,
        machine_params,
        core_params,
        dcache_params,
        icache_params,
        l2_params,
        l1dir_params,
        l2dir_params,
    ):
        super().__init__(machine_params)
        self._defined_config_params = {
            "num_cores",
            "num_l2s",
            "private_l2",
            "num_l1_dirs",
            "num_l2_dirs",
            "num_nocs",
            "tech_node",
            "interconnect_type",
            "temperature",
            "device_type",
            "longer_chan_dev",
            "embedded",
            "machine_bits",
            "phy_addr_width",
            "virt_addr_width",
            "temperature",
            "vm_pg_size",
        }
        # Optional McPAT user Vdd override (XML core/L2 `vdd` param; see
        # CactiTechParams.__init__). Accepted only when present, so existing
        # callers that never pass it are validated exactly as before.
        if "vdd" in machine_params:
            self._defined_config_params.add("vdd")
        self._core_params = core_params
        self.validate()
        self.reconfigure_params()
        self.add_cores()

    def reconfigure_params(self):
        self._config_params["data_width"] = (
            int(ceil(self._config_params["machine_bits"] / 32)) * 32
        )

    def add_cores(self):
        core_cfg = McPATCoreConfig(self._core_params)
        self._config_params["core0"] = core_cfg._config_params
        self._defined_config_params.add("core0")
