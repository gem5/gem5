from itertools import zip_longest

import m5.stats
from m5.objects import Root
from m5.util import panic


class _DeltaStat:
    pass


class AbstractPowerModel:
    def __init__(
        self, simobj, interval=0, interval_ticks=0, sample_clock=None
    ):
        self._simobj = simobj
        self.name = "AbstractPowerModel"
        self._interval = interval
        self._interval_ticks = interval_ticks
        self._stats = {}
        self._sample_stats_prepared = False
        self._sample_mode = False
        # Opt-in {requested: read_instead} stat aliases, e.g. serving FP
        # stats from vector counterparts when scalar FP lives in vector regs.
        self._stat_alias = {}
        # PowerModelPyFunc whose getSampleDurationSeconds() times a sample;
        # None falls back to interval ticks, then numCycles.
        self._sample_clock = sample_clock

    def set_stat_aliases(self, aliases):
        self._stat_alias = dict(aliases)

    def set_sample_clock(self, clock):
        self._sample_clock = clock
        for attr in getattr(self, "STATIC_CHILDREN", ()):
            child = getattr(self, attr, None)
            if child is not None:
                child.set_sample_clock(clock)

    def sampling_enabled(self) -> bool:
        return self._interval > 0 or self._interval_ticks > 0

    def set_sample_mode(self, enable: bool):
        self._sample_mode = enable
        if enable:
            self._sample_stats_prepared = False

    def clear_sample_state(self):
        """
        Clear Python-side delta bookkeeping.

        This must be called at ROI start after m5.stats.reset(), otherwise the
        next sampled delta can be computed against a pre-reset stat baseline.
        """
        self._stats.clear()
        self._sample_stats_prepared = False
        self._sample_mode = False

    def _snapshot_stat(self, stat):
        if hasattr(stat, "value"):
            value = stat.value
            if isinstance(value, list):
                if not value:
                    # gem5 Formula stats have an EMPTY .value; .total is
                    # the real cumulative count.
                    return stat.total
                return list(value)
            return value
        return stat.total

    def _delta_value(self, current, previous):
        if isinstance(current, list):

            return [
                curr - prev
                for curr, prev in zip_longest(current, previous, fillvalue=0.0)
            ]
        return current - previous

    def _build_delta_stat(self, stat, value):
        delta_stat = _DeltaStat()
        if isinstance(value, list):
            delta_stat.value = value
            delta_stat.total = sum(value)
            if hasattr(stat, "subnames"):
                delta_stat.subnames = stat.subnames
            if hasattr(stat, "ysubnames"):
                delta_stat.ysubnames = stat.ysubnames
        else:
            delta_stat.value = value
            delta_stat.total = value
        return delta_stat

    def _prepare_stats_for_sample(self):
        if not self.sampling_enabled() or self._sample_stats_prepared:
            return

        sim_root = Root.getInstance()
        if sim_root:
            sim_root.preDumpStats()

        self._sample_stats_prepared = True

    def get_stat(self, stat):
        if stat in self._stat_alias:
            stat = self._stat_alias[stat]
        try:
            stat_info = self._simobj.resolveStat(stat)
            if not stat_info:
                panic(f"{stat} not found in stats!")
                return 0.0

            if not self.sampling_enabled() or not self._sample_mode:
                stat_info.prepare()
                return stat_info

            self._prepare_stats_for_sample()
            stat_info.prepare()
            value = self._snapshot_stat(stat_info)

            if stat not in self._stats:
                self._stats[stat] = {
                    "value": value,
                    "active": 1,
                    "delta": value,
                }
            elif self._stats[stat]["active"] == 0:
                prev = self._stats[stat]["value"]
                self._stats[stat]["value"] = value
                self._stats[stat]["active"] = 1
                self._stats[stat]["delta"] = self._delta_value(value, prev)

            return self._build_delta_stat(
                stat_info, self._stats[stat]["delta"]
            )
        except KeyError:
            panic(f"{stat} not found in stats!")
            return 0.0

    def reset_stats_dict(self):
        for stat in self._stats.values():
            stat["active"] = 0
        self._sample_stats_prepared = False

    def dynamic_power(self) -> float:
        raise NotImplementedError

    def static_power(self, temp_k) -> float:
        """Static power in Watts at `temp_k` (Kelvin)."""
        raise NotImplementedError

    def getExecutionTime(self):
        """
        Return the denominator used to convert event energy to Watts.

        Normal stats path:
            Use full simulated/stat window time.

        Interval-sampling path:
            Use the actual active sample duration, but only while the
            PowerModelPyFunc C++ wrapper is actively evaluating an interval
            sample.
        """
        if getattr(self, "_sample_mode", False):
            if self._sample_clock is not None:
                dur = self._sample_clock.getSampleDurationSeconds()
                if dur > 0:
                    return dur

            if self._interval_ticks > 0:
                return self._interval_ticks / 1e12

            clk_domain = self._simobj.clk_domain.clock.getValue()[0]
            try:
                cycles = self.get_stat("numCycles").total
            except Exception:
                cycles = self._interval
            return cycles / (1e12 / clk_domain)

        return Root.getInstance().resolveStat("simSeconds").total

    def convert_to_watts(self, value: float) -> float:
        time = self.getExecutionTime()
        if time == 0:
            return 0.0
        value_in_j = value * 1e-9
        return value_in_j / time
