from m5.objects import (
    BaseO3CPU,
    Root,
)

# Never use `import *`
from .base_power_model import AbstractPowerModel
from .mcpat_power_model import McPATPowerModel


class O3McPATRenamingUnitPower(McPATPowerModel):
    STATIC_UNITS = ("IntFRAT", "FpFRAT", "IntFreeList", "FpFreeList")
    STATIC_PIPELINE_SHARE = True

    def __init__(
        self,
        o3cpu: BaseO3CPU,
        act_energies,
        pipeline_act_factor,
        interval=0,
        interval_ticks=0,
        *,
        machine_model,
    ):
        super().__init__(
            o3cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "O3McPATRenamingUnitPower"

        """
         McPAT supports PRF and RS based renaming, but gem5 only renames using
         a PRF combined with a free list.
        """
        """
         McPAT also supports two renaming SCHEMES (rename_scheme):
         CAM-based (an associative RAT search, core.cc:1420-1482) and
         RAM-based (a direct-indexed RAT read, core.cc:1366-1419). gem5's own
         SimpleRenameMap/free-list mechanism is RAM-based (direct-indexed,
         no associative search -- confirmed directly from gem5's source),
         so int_frat_energy()/fp_frat_energy() below use the RAM-based
         formula (reads*(Read+DCL) + writes*Write, core.cc:2614-2625) --
         NOT the CAM-based one (reads*(Search+DCL) + writes*Write,
         core.cc:2626-2637) this file used before. The activation-energy
         dict fed in must be built from a RAM-based (assoc=1, "(RAM)"-
         suffixed) FRAT array to match -- see
         mcpat_solver_bridge_live.py's _TRACE_NAME_FOR.
        """
        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor
        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        int_frat = self.int_frat_energy()
        fp_frat = self.fp_frat_energy()
        int_fl = self.int_fl_energy()
        fp_fl = self.fp_fl_energy()
        total_energy = (
            int_frat + fp_frat + int_fl + fp_fl + self.rnu_pipeline_energy()
        )
        print(" " * indent + f"Renaming Unit:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Int Front End RAT:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_frat)} W\n"
        )
        print(" " * (indent + 4) + f"FP Front End RAT:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_frat)} W\n"
        )
        print(" " * (indent + 4) + f"Int Free List:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_fl)} W\n"
        )
        print(" " * (indent + 4) + f"FP Free List:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_fl)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.int_frat_energy()
        energy += self.fp_frat_energy()
        energy += self.int_fl_energy()
        energy += self.fp_fl_energy()
        energy += self.rnu_pipeline_energy()
        return self.convert_to_watts(energy)

    def int_frat_energy(self) -> float:
        int_rename_reads = self.get_stat("rename.intLookups").total
        int_rename_writes = self.get_stat("rename.intReturned").total

        return (
            int_rename_reads
            * (
                self._act_energies["IntFRAT"]["Lookup"]
                + self._act_energies["IntDCL"]
            )
            + int_rename_writes * self._act_energies["IntFRAT"]["Write"]
        ) + self._local_read("IntFRAT")

    def fp_frat_energy(self) -> float:
        fp_rename_reads = self.get_stat("rename.fpLookups").total
        fp_rename_writes = self.get_stat("rename.fpReturned").total

        return (
            fp_rename_reads
            * (
                self._act_energies["FpFRAT"]["Lookup"]
                + self._act_energies["FpDCL"]
            )
            + fp_rename_writes * self._act_energies["FpFRAT"]["Write"]
        ) + self._local_read("FpFRAT")

    def int_fl_energy(self) -> float:
        int_rename_reads = self.get_stat("rename.intLookups").total
        int_rename_writes = self.get_stat("rename.intReturned").total
        return (
            int_rename_reads * self._act_energies["IntFreeList"]["Read"]
            + 2
            * int_rename_writes
            * self._act_energies["IntFreeList"]["Write"]
        ) + self._local_read("IntFreeList")

    def fp_fl_energy(self) -> float:
        fp_rename_reads = self.get_stat("rename.fpLookups").total
        fp_rename_writes = self.get_stat("rename.fpReturned").total
        return (
            fp_rename_reads * self._act_energies["FpFreeList"]["Read"]
            + 2 * fp_rename_writes * self._act_energies["FpFreeList"]["Write"]
        ) + self._local_read("FpFreeList")

    def _local_read(self, key) -> float:
        # McPAT adds one array-read energy per rename leaf whatever the
        # activity (core.cc:2764-2769).
        return self._act_energies[key]["Read"]

    def rnu_pipeline_energy(self) -> float:
        cycles = self.get_stat("numCycles").total
        rtp_pipeline_coe = self._pipeline_act_factor * cycles
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]
