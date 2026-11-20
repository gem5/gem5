"""L1 instruction-cache McPAT power model."""

from mcpat_power_model.o3_mcpat_power_model.mcpat_power_model import (
    McPATPowerModel,
)


class L1IMcPATPower(McPATPowerModel):
    STATIC_BUCKET = "Instruction Cache"
    BUFFERS = ("icacheMissBuffer", "icacheFillBuffer", "icacheprefetchBuffer")
    STATIC_UNITS = ("Data", "Tag") + BUFFERS

    def __init__(
        self,
        l1icache,
        act_energies,
        interval=0,
        interval_ticks=0,
        *,
        machine_model,
        sample_clock=None,
    ):
        super().__init__(
            l1icache,
            act_energies,
            interval,
            interval_ticks,
            machine_model,
            sample_clock,
        )
        self.name = "L1IMcPATPower"

    def dynamic_power(self):
        return self.convert_to_watts(self.icache_energy())

    def icache_energy(self):
        """core.cc:2198-2213: data+tag read per access; per miss a line
        refill and one search + write on every buffer."""
        ae = self._act_energies
        reads = self.get_stat("overallAccesses").total
        misses = self.get_stat("overallMisses").total
        buf_e = sum(
            ae[b]["Search"] + ae[b]["Write"] for b in self.BUFFERS if b in ae
        )
        e = reads * (ae["Read"] + ae["TagRead"])
        e += misses * (ae["Write"] + ae["TagWrite"])
        e += misses * buf_e
        return e
