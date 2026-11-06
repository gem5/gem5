"""L2 shared-cache McPAT power model."""

from mcpat_power_model.o3_mcpat_power_model.mcpat_power_model import (
    McPATPowerModel,
)

# gem5 stat paths on the stdlib L2 ``Cache`` SimObject, matching McPAT
# ``sharedcache.cc:663-670`` (the L2 runtime-power branch consumes only
# ``read_accesses``/``read_misses``/``write_accesses``/``write_misses``).
# At a shared L2 behind an ``L2XBar`` the plain ``ReadReq``/``WriteReq``
# MemCmds are dead (write-back L1s: no write-through; demand reads arrive as
# ``ReadShared``/``ReadEx``/``ReadClean``, all rolled up in
# ``overall{Hits,Misses}``). Writes into the L2 are L1D writeback evictions:
# ``WritebackDirty`` (dirty lines) + ``WritebackClean`` (clean lines, when
# the L1 forwards them -- inclusive L2 / ``writeback_clean=True``). On a
# write-back L1D that drops clean evictions, ``WritebackClean.*`` resolves
# to a live 0 -- summed in anyway so a hierarchy with nonzero clean
# writebacks is not silently under-counted. Cross-check:
# ``WritebackDirty.accesses == L1D writebacks::total`` exactly. Kept as
# constants so a correction stays a small edit; this mapping is applied in
# lockstep by ``validate_daxpy_power.py``'s ``gather_stats()``.
_STAT_READ_HITS = "overallHits"
_STAT_READ_MISSES = "overallMisses"
_STAT_WRITE_ACCESSES = ("WritebackDirty.accesses", "WritebackClean.accesses")
_STAT_WRITE_MISSES = ("WritebackDirty.misses", "WritebackClean.misses")


class L2McPATPower(McPATPowerModel):
    STATIC_BUCKET = "L2"
    BUFFERS = ("L2MissB", "L2FillB", "L2PrefetchB", "L2WBB")
    STATIC_UNITS = ("Data", "Tag") + BUFFERS

    def __init__(
        self,
        l2cache,
        act_energies,
        interval=0,
        interval_ticks=0,
        *,
        machine_model,
        sample_clock=None,
    ):
        super().__init__(
            l2cache,
            act_energies,
            interval,
            interval_ticks,
            machine_model,
            sample_clock,
        )
        self.name = "L2McPATPower"

    def dynamic_power(self):
        return self.convert_to_watts(self.l2_energy())

    def l2_energy(self):
        """sharedcache.cc:763-767 array + 789-797 write-back buffers.
        write_accesses is the L1D writeback count, not hits + misses."""
        ae = self._act_energies
        read_hits = self.get_stat(_STAT_READ_HITS).total
        read_misses = self.get_stat(_STAT_READ_MISSES).total
        writes = sum(self.get_stat(s).total for s in _STAT_WRITE_ACCESSES)
        write_misses = sum(self.get_stat(s).total for s in _STAT_WRITE_MISSES)
        buf_e = sum(
            ae[b]["Search"] + ae[b]["Write"] for b in self.BUFFERS if b in ae
        )
        e = (
            read_hits * (ae["Read"] + ae["TagRead"])
            + read_misses * ae["TagRead"]
            + write_misses * ae["TagWrite"]
            + writes * (ae["Write"] + ae["TagWrite"])
        )
        e += write_misses * buf_e
        return e
