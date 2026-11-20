"""L1 data-cache McPAT power model."""

from mcpat_power_model.o3_mcpat_power_model.mcpat_power_model import (
    McPATPowerModel,
)


class L1DMcPATPower(McPATPowerModel):
    STATIC_BUCKET = "Data Cache"
    BUFFERS = (
        "dcacheMissBuffer",
        "dcacheFillBuffer",
        "dcacheprefetchBuffer",
        "dcacheWBB",
    )
    STATIC_UNITS = ("Data", "Tag") + BUFFERS

    def __init__(
        self,
        l1dcache,
        act_energies,
        interval=0,
        interval_ticks=0,
        *,
        machine_model,
        sample_clock=None,
    ):
        super().__init__(
            l1dcache,
            act_energies,
            interval,
            interval_ticks,
            machine_model,
            sample_clock,
        )
        self.name = "L1DMcPATPower"
        policy = getattr(machine_model, "write_policy", None)
        config = (
            getattr(machine_model, "resolved_config", {})
            .get("cache_configs", {})
            .get("dcache_config")
        )
        self._write_back = (
            (config is None or config[7] == 1)
            if policy is None
            else policy == "write_back"
        )

    def dynamic_power(self):
        return self.convert_to_watts(self.dcache_energy())

    def dcache_energy(self):
        """core.cc:3316-3336 (Write_back): data+tag read/write per access;
        per write miss a tag read, the deferred write-back and one
        search + write on every buffer."""
        ae = self._act_energies
        reads = sum(
            self.get_stat(f"ReadReq.{s}").total for s in ("hits", "misses")
        )
        writes = sum(
            self.get_stat(f"WriteReq.{s}").total for s in ("hits", "misses")
        )
        write_misses = self.get_stat("WriteReq.misses").total
        # A write-through D$ has no dcacheWBB.
        buf_e = sum(
            ae[b]["Search"] + ae[b]["Write"] for b in self.BUFFERS if b in ae
        )
        write_e = ae["Write"] + ae["TagWrite"]
        e = reads * (ae["Read"] + ae["TagRead"]) + writes * write_e
        e += write_misses * ae["TagRead"]
        if self._write_back:
            e += write_misses * write_e
            e += write_misses * buf_e
        else:
            read_misses = self.get_stat("ReadReq.misses").total
            e += read_misses * buf_e
        return e
