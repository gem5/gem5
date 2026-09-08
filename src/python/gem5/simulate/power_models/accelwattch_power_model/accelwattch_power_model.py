from m5.objects import (
    MemCtrl,
    PowerModel,
    PowerModelPyFunc,
    Shader,
)

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel
from .gpu_caches_power_model.accelwattch_dcache_power_model import (
    AccelwattchDataCachePowerModel,
)
from .gpu_caches_power_model.accelwattch_icache_power_model import (
    AccelwattchInstCachePowerModel,
)
from .gpu_caches_power_model.accelwattch_l2cache_power_model import (
    AccelwattchL2CachePowerModel,
)
from .gpu_caches_power_model.accelwattch_noc_power_model import (
    AccelwattchNoCPowerModel,
)
from .gpu_memory_power_model.accelwattch_dram_power_model import (
    AccelwattchMemDRAMPowerModel,
)
from .gpu_memory_power_model.accelwattch_memory_ctrl_power_model import (
    AccelwattchMemCtrlPowerModel,
)
from .gpu_power_model.accelwattch_alu_power_model import (
    AccelwattchALUPowerModel,
)
from .gpu_power_model.accelwattch_fpu_power_model import (
    AccelwattchFPUPowerModel,
)
from .gpu_power_model.accelwattch_idle_power_model import (
    AccelwattchIdlePowerModel,
)
from .gpu_power_model.accelwattch_inst_buffer_power_model import (
    AccelwattchInstBufferPowerModel,
)
from .gpu_power_model.accelwattch_lds_power_model import (
    AccelwattchLDSPowerModel,
)
from .gpu_power_model.accelwattch_pipeline_power_model import (
    AccelwattchPipelinePowerModel,
)
from .gpu_power_model.accelwattch_reg_file_power_model import (
    AccelwattchRegisterFilePowerModel,
)
from .gpu_power_model.accelwattch_scheduler_power_model import (
    AccelwattchSchedulerPowerModel,
)
from .gpu_power_model.accelwattch_sfu_power_model import (
    AccelwattchSFUPowerModel,
)


class AccelwattchPowerOn(PowerModelPyFunc):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__()
        self._scaling_factors = scaling_factors
        self.interval = interval
        self.clock_stat = "shaderActiveCycles"
        self._ib = AccelwattchInstBufferPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._rf = AccelwattchRegisterFilePowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._lds = AccelwattchLDSPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._alu = AccelwattchALUPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._fpu = AccelwattchFPUPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._sfu = AccelwattchSFUPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._schedu = AccelwattchSchedulerPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._pipeline = AccelwattchPipelinePowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._idle = AccelwattchIdlePowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        """ Memory (DRAM + Ctrl) Power Models """
        self._mem_ctrl = AccelwattchMemCtrlPowerModel(
            gpu=gpu,
            gpu_memory=gpu_memory,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._dram = AccelwattchDRAMPowerModel(
            gpu=gpu,
            gpu_memory=gpu_memory,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )

        """ Cache + NoC Power Models """
        self._icache = AccelwattchInstCachePowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._dcache = AccelwattchDataCachePowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._l2cache = AccelwattchL2CachePowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )
        self._noc = AccelwattchNoCPowerModel(
            gpu=gpu,
            act_energies=act_energies,
            scaling_factors=scaling_factors,
            interval=interval,
        )

    def dynamic_power(self):
        power = (
            self._ib.dynamic_power()
            + self._schedu.dynamic_power()
            + self._icache.dynamic_power()
            + self._dcache.dynamic_power()
            + self._lds.dynamic_power()
            + self._rf.dynamic_power()
            + self._alu.dynamic_power()
            + self._fpu.dynamic_power()
            + self._sfu.dynamic_power()
            + self._mem_ctrl.dynamic_power()
            + self._dram.dynamic_power()
            + self._noc.dynamic_power()
            + self._l2cache.dynamic_power()
            + self._pipeline.dynamic_power()
            + self._idle.dynamic_power()
            + self._scaling_factors["CONSTANT_POWER"]
        )
        return power

    def static_power(self):
        int_add_accesses = self._alu.iadd_static_coeff()
        int_mul_accesses = self._sfu.imul_static_coeff()
        int_accesses = int_add_accesses + int_mul_accesses

        fp_accesses = (
            self._fpu.fp_static_coeff() + self._sfu.fpmul_static_coeff()
        )
        dp_accesses = (
            self._fpu.dp_static_coeff() + self._sfu.dpmul_static_coeff()
        )
        sfu_accesses = self._sfu.sfu_static_coeff()
        tensor_accesses = self._sfu.tensor_static_coeff()
        tex_accesses = self._sfu.tex_static_coeff()

        l1_accesses = self._dcache.dcache_static_coeff()
        l2_accesses = self._l2cache.l2cache_static_coeff()
        lds_accesses = self._lds.lds_static_coeff()
        active_cores = self._alu.get_avg_active_cores()
        avg_threads_in_wf = self._alu.get_avg_threads_in_wf()

        base_static_power = 0.0
        lane_static_power = 0.0

        if avg_threads_in_wf == 0:
            if l1_accesses != 0:
                return self._scaling_factors["static_l1_flane"] * active_cores
            elif lds_accesses != 0:
                return (
                    self._scaling_factors["static_shared_flane"] * active_cores
                )
            elif l2_accesses != 0:
                return self._scaling_factors["static_l2_flane"] * active_cores
            else:
                return (
                    self._scaling_factors["static_light_flane"] * active_cores
                )

        if (
            (int_accesses != 0)
            and (fp_accesses != 0)
            and (dp_accesses != 0)
            and (sfu_accesses == 0)
            and (tensor_accesses == 0)
            and (tex_accesses == 0)
        ):
            base_static_power = self._scaling_factors["static_cat3_flane"]
            lane_static_power = self._scaling_factors["static_cat3_addlane"]
        elif (
            (int_accesses != 0)
            and (fp_accesses != 0)
            and (dp_accesses == 0)
            and (sfu_accesses == 0)
            and (tensor_accesses != 0)
            and (tex_accesses == 0)
        ):
            base_static_power = self._scaling_factors["static_cat6_flane"]
            lane_static_power = self._scaling_factors["static_cat6_addlane"]
        elif (
            (int_accesses != 0)
            and (fp_accesses != 0)
            and (dp_accesses == 0)
            and (sfu_accesses != 0)
            and (tensor_accesses == 0)
            and (tex_accesses == 0)
        ):
            base_static_power = self._scaling_factors["static_cat4_flane"]
            lane_static_power = self._scaling_factors["static_cat4_addlane"]
        elif (
            (int_accesses != 0)
            and (fp_accesses != 0)
            and (dp_accesses == 0)
            and (sfu_accesses == 0)
            and (tensor_accesses == 0)
            and (tex_accesses != 0)
        ):
            base_static_power = self._scaling_factors["static_cat5_flane"]
            lane_static_power = self._scaling_factors["static_cat5_addlane"]
        elif (
            (int_accesses != 0)
            and (fp_accesses != 0)
            and (dp_accesses == 0)
            and (sfu_accesses == 0)
            and (tensor_accesses == 0)
            and (tex_accesses == 0)
        ):
            base_static_power = self._scaling_factors["static_cat2_flane"]
            lane_static_power = self._scaling_factors["static_cat2_addlane"]
        elif (
            (int_accesses != 0)
            and (fp_accesses == 0)
            and (dp_accesses == 0)
            and (sfu_accesses == 0)
            and (tensor_accesses == 0)
            and (tex_accesses == 0)
        ):
            if (int_add_accesses != 0) and (int_mul_accesses == 0):
                base_static_power = self._scaling_factors[
                    "static_intadd_flane"
                ]
                lane_static_power = self._scaling_factors[
                    "static_intadd_addlane"
                ]
            elif (int_add_accesses == 0) and (int_mul_accesses != 0):
                base_static_power = self._scaling_factors[
                    "static_intmul_flane"
                ]
                lane_static_power = self._scaling_factors[
                    "static_intmul_addlane"
                ]
            else:
                base_static_power = self._scaling_factors["static_cat1_flane"]
                lane_static_power = self._scaling_factors[
                    "static_cat1_addlane"
                ]
        elif (
            (int_accesses == 0)
            and (fp_accesses == 0)
            and (dp_accesses == 0)
            and (sfu_accesses == 0)
            and (tensor_accesses == 0)
            and (tex_accesses == 0)
        ):
            lane_static_power = 0
            if l1_accesses != 0:
                base_static_power = self._scaling_factors["static_l1_flane"]
            elif lds_accesses != 0:
                base_static_power = self._scaling_factors[
                    "static_shared_flane"
                ]
            elif l2_accesses != 0:
                base_static_power = self._scaling_factors["static_l2_flane"]
            else:
                base_static_power = self._scaling_factors["static_light_flane"]
                lane_static_power = self._scaling_factors[
                    "static_light_addlane"
                ]
        else:
            base_static_power = self._scaling_factors["static_geomean_flane"]
            lane_static_power = self._scaling_factors["static_geomean_addlane"]

        total_static_power = (
            base_static_power + avg_threads_in_wf * lane_static_power
        ) * active_cores
        self.reset_stats_dict()
        return total_static_power

    def reset_stats_dict(self):
        self._ib.reset_stats_dict()
        self._schedu.reset_stats_dict()
        self._icache.reset_stats_dict()
        self._dcache.reset_stats_dict()
        self._lds.reset_stats_dict()
        self._rf.reset_stats_dict()
        self._alu.reset_stats_dict()
        self._fpu.reset_stats_dict()
        self._sfu.reset_stats_dict()
        self._pipeline.reset_stats_dict()
        self._mem_ctrl.reset_stats_dict()
        self._dram.reset_stats_dict()
        self._l2cache.reset_stats_dict()
        self._idle.reset_stats_dict()


class AccelwattchPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class AccelwattchPowerModel(PowerModel):
    def __init__(
        self, gpu, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            AccelwattchPowerOn(
                gpu, gpu_memory, act_energies, scaling_factors, interval
            ),  # ON
            AccelwattchPowerOff(),  # CLK_GATED
            AccelwattchPowerOff(),  # SRAM_RETENTION
            AccelwattchPowerOff(),  # OFF
        ]
