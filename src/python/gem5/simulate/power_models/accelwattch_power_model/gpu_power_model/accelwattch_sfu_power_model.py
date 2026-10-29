from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchSFUPowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchSFUPowerModel"

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        """ For below:
            Note that the scaling factors are currently
            using SASS values from GPGPU-Sim, and using
            SASS, you do not get the granularity of
            intMul24/32 Insts.
        """
        energy = (
            self.int_mul_energy()
            + self.int_mul24_energy()
            + self.int_mul32_energy()
            + self.int_div_energy()
            + self.fp_mul_energy()
            + self.fp_div_energy()
            + self.fp_sqrt_energy()
            + self.fp_lg_energy()
            + self.fp_sin_energy()
            + self.fp_exp_energy()
            + self.dp_mul_energy()
            + self.dp_div_energy()
            + self.tensor_energy()
            + self.texture_energy()
        )
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def imul_static_coeff(self) -> float:
        tot_accesses = self.total_sfu_accesses()
        imul_acc = (
            sum(
                self.get_stat(f"CUs{i}.intMulInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_MUL_ACC"]
        )
        imul24_acc = (
            sum(
                self.get_stat(f"CUs{i}.intMul24Insts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_MUL24_ACC"]
        )
        imul32_acc = (
            sum(
                self.get_stat(f"CUs{i}.intMul32Insts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_MUL32_ACC"]
        )
        idiv_acc = (
            sum(
                self.get_stat(f"CUs{i}.intDivInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_DIV_ACC"]
        )
        if tot_accesses == 0:
            return 0
        coeff = (
            self._act_energies["SFU"]
            * (imul_acc + imul24_acc + imul32_acc + idiv_acc)
            / tot_accesses
        )
        return self.convert_to_watts(coeff)

    def fpmul_static_coeff(self) -> float:
        tot_accesses = self.total_sfu_accesses()
        fpmul_acc = (
            sum(
                self.get_stat(f"CUs{i}.fpMulInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_MUL_ACC"]
        )
        fpdiv_acc = (
            sum(
                self.get_stat(f"CUs{i}.fpDivInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_DIV_ACC"]
        )
        if tot_accesses == 0:
            return 0
        coeff = (
            self._act_energies["SFU"] * (fpmul_acc + fpdiv_acc) / tot_accesses
        )
        return self.convert_to_watts(coeff)

    def dpmul_static_coeff(self) -> float:
        tot_accesses = self.total_sfu_accesses()
        dpmul_acc = (
            sum(
                self.get_stat(f"CUs{i}.dpMulInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["DP_MUL_ACC"]
        )
        dpdiv_acc = (
            sum(
                self.get_stat(f"CUs{i}.dpDivInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["DP_DIV_ACC"]
        )
        if tot_accesses == 0:
            return 0
        coeff = (
            self._act_energies["SFU"] * (dpmul_acc + dpdiv_acc) / tot_accesses
        )
        return self.convert_to_watts(coeff)

    def sfu_static_coeff(self) -> float:
        tot_accesses = self.total_sfu_accesses()
        sqrt_acc = (
            sum(
                self.get_stat(f"CUs{i}.fpSqrtInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_SQRT_ACC"]
        )
        lg_acc = (
            sum(
                self.get_stat(f"CUs{i}.fpLgInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_LG_ACC"]
        )
        sin_acc = (
            sum(
                self.get_stat(f"CUs{i}.fpSinInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_SIN_ACC"]
        )
        exp_acc = (
            sum(
                self.get_stat(f"CUs{i}.fpExpInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_EXP_ACC"]
        )

        if tot_accesses == 0:
            return 0
        coeff = (
            self._act_energies["SFU"]
            * (sqrt_acc + lg_acc + sin_acc + exp_acc)
            / tot_accesses
        )
        return self.convert_to_watts(coeff)

    def tensor_static_coeff(self) -> float:
        tot_accesses = self.total_sfu_accesses()
        tensor_acc = (
            sum(
                self.get_stat(f"CUs{i}.numVecOpsExecutedMFMA")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["TENSOR_ACC"]
        )
        if tot_accesses == 0:
            return 0
        coeff = self._act_energies["SFU"] * (tensor_acc) / tot_accesses
        return self.convert_to_watts(coeff)

    def tex_static_coeff(self) -> float:
        tot_accesses = self.total_sfu_accesses()
        tex_acc = (
            sum(
                self.get_stat(f"CUs{i}.texInsts") for i in range(self._num_cus)
            )
            * self._scaling_factors["TEX_ACC"]
        )
        if tot_accesses == 0:
            return 0
        coeff = self._act_energies["SFU"] * (tex_acc) / tot_accesses
        return self.convert_to_watts(coeff)

    def total_sfu_accesses(self) -> float:
        accelsim_coeff = [
            "INT_MUL_ACC",
            "INT_MUL24_ACC",
            "INT_MUL32_ACC",
            "INT_DIV_ACC",
            "FP_MUL_ACC",
            "FP_DIV_ACC",
            "FP_SQRT_ACC",
            "FP_LG_ACC",
            "FP_SIN_ACC",
            "FP_EXP_ACC",
            "DP_MUL_ACC",
            "DP_DIV_ACC",
            "TENSOR_ACC",
            "TEX_ACC",
        ]
        gem5_stats = [
            "intMulInsts",
            "intMul24Insts",
            "intMul32Insts",
            "intDivInsts",
            "fpMulInsts",
            "fpDivInsts",
            "fpSqrtInsts",
            "fpLgInsts",
            "fpSinInsts",
            "fpExpInsts",
            "dpMulInsts",
            "dpDivInsts",
            "numVecOpsExecutedMFMA",
            "texInsts",
        ]
        accesses = 0
        for stat, coeff in zip(gem5_stats, accelsim_coeff):
            accesses = accesses + (
                sum(
                    self.get_stat(f"CUs{i}.{stat}")
                    for i in range(self._num_cus)
                )
                * self._scaling_factors[coeff]
            )

        return accesses

    def int_mul_energy(self) -> float:
        int_mul_accesses = (
            sum(
                self.get_stat(f"CUs{i}.intMulInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_MUL_ACC"]
        )
        return int_mul_accesses * self._act_energies["SFU"]

    def int_mul24_energy(self) -> float:
        int_mul24_accesses = (
            sum(
                self.get_stat(f"CUs{i}.intMul24Insts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_MUL24_ACC"]
        )
        return int_mul24_accesses * self._act_energies["SFU"]

    def int_mul32_energy(self) -> float:
        int_mul32_accesses = (
            sum(
                self.get_stat(f"CUs{i}.intMul32Insts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_MUL32_ACC"]
        )
        return int_mul32_accesses * self._act_energies["SFU"]

    def int_div_energy(self) -> float:
        int_div_accesses = (
            sum(
                self.get_stat(f"CUs{i}.intDivInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_DIV_ACC"]
        )
        return int_div_accesses * self._act_energies["SFU"]

    def fp_mul_energy(self) -> float:
        fp_mul_accesses = (
            sum(
                self.get_stat(f"CUs{i}.fpMulInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_MUL_ACC"]
        )
        return fp_mul_accesses * self._act_energies["SFU"]

    def fp_div_energy(self) -> float:
        fp_div_accesses = (
            sum(
                self.get_stat(f"CUs{i}.fpDivInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_DIV_ACC"]
        )
        return fp_div_accesses * self._act_energies["SFU"]

    def fp_sqrt_energy(self) -> float:
        fp_sqrt_accesses = (
            sum(
                self.get_stat(f"CUs{i}.fpSqrtInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_SQRT_ACC"]
        )
        return fp_sqrt_accesses * self._act_energies["SFU"]

    def fp_lg_energy(self) -> float:
        fp_lg_accesses = (
            sum(
                self.get_stat(f"CUs{i}.fpLgInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_LG_ACC"]
        )
        return fp_lg_accesses * self._act_energies["SFU"]

    def fp_sin_energy(self) -> float:
        fp_sin_accesses = (
            sum(
                self.get_stat(f"CUs{i}.fpSinInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_SIN_ACC"]
        )
        return fp_sin_accesses * self._act_energies["SFU"]

    def fp_exp_energy(self) -> float:
        fp_exp_accesses = (
            sum(
                self.get_stat(f"CUs{i}.fpExpInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_EXP_ACC"]
        )
        return fp_exp_accesses * self._act_energies["SFU"]

    def dp_mul_energy(self) -> float:
        dp_mul_accesses = (
            sum(
                self.get_stat(f"CUs{i}.dpMulInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["DP_MUL_ACC"]
        )
        return dp_mul_accesses * self._act_energies["SFU"]

    def dp_div_energy(self) -> float:
        dp_div_accesses = (
            sum(
                self.get_stat(f"CUs{i}.dpDivInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["DP_DIV_ACC"]
        )
        return dp_div_accesses * self._act_energies["SFU"]

    def tensor_energy(self) -> float:
        """NOTE: Tensor instructions/operations are MFMA Ops,
        in AMD notation.
        """
        tensor_accesses = (
            sum(
                self.get_stat(f"CUs{i}.numVecOpsExecutedMFMA")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["TENSOR_ACC"]
        )
        return tensor_accesses * self._act_energies["SFU"]

    def texture_energy(self) -> float:
        texture_accesses = (
            sum(
                self.get_stat(f"CUs{i}.texInsts") for i in range(self._num_cus)
            )
            * self._scaling_factors["TEX_ACC"]
        )
        return texture_accesses * self._act_energies["SFU"]
