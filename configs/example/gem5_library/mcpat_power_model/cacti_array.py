# SPDX-License-Identifier: BSD-3-Clause
"""Standalone CACTI interface, independent of gem5 and McPAT composition."""

from dataclasses import (
    asdict,
    dataclass,
    replace,
)
from functools import lru_cache

from .coefficients import StaticCoefficients
from .mcpat_solver.array_spec import build_array
from .mcpat_solver.cacti_dynamic_params import CactiArrayConfig
from .mcpat_solver.cacti_partition_search import (
    enumerate_side,
    optimize_array_partition,
)
from .mcpat_solver.records import (
    ArrayConfig,
    Partition,
    SearchOptions,
    Technology,
)


@dataclass(frozen=True)
class ArrayResult:
    activation_energies: dict
    static_coeffs: StaticCoefficients
    partitions: dict
    timing: dict | None


class CactiArray:
    """Construction stores inputs; build evaluates candidates; search selects."""

    def __init__(self, config, *, technology=None, search_options=None):
        self.config = config
        self.technology = technology or Technology()
        self.search_options = search_options or SearchOptions()
        self.result = None
        self._built = False

    def _config(self, is_tag=None, *, rebuild=False):
        c, t = self.config, self.technology
        inside = (
            (0 if t.embedded else 2)
            if c.wire_inside is None
            else c.wire_inside
        )
        outside = inside if c.wire_outside is None else c.wire_outside
        overhead = (
            (30 if t.embedded else 0)
            if c.wire_overhead is None
            else c.wire_overhead
        )
        tag = c.is_tag if is_tag is None else is_tag
        return CactiArrayConfig(
            capacity=c.capacity,
            block_sz=c.line_bytes,
            out_w=c.data_width,
            assoc=c.associativity,
            nbanks=c.banks,
            tag_w=c.tag_width,
            specific_tag=bool(c.tag_width) if tag or not c.has_tag else False,
            is_cache=rebuild,
            pure_cam=c.pure_cam,
            is_tag=tag,
            add_ecc=c.add_ecc,
            data_assoc=c.data_associativity,
            is_seq_acc=c.sequential_access,
            fast_access=c.fast_access,
            num_rw_ports=c.rw_ports,
            num_rd_ports=c.read_ports,
            num_wr_ports=c.write_ports,
            num_search_ports=c.search_ports,
            wire_is_mat_type=inside,
            wire_os_mat_type=outside,
            wt_overhead=overhead,
        )

    def build(self):
        if self._built:
            return self
        self.config.validate()
        self.search_options.validate()
        self._params = self.technology.parameters()
        if self.search_options.partition is None:
            cfg = self._config()
            nspd = 1.0 if cfg.assoc == 0 else cfg.out_w / (cfg.block_sz * 8.0)
            self.candidates = enumerate_side(cfg, self._params, nspd)[0]
            if self.config.has_tag:
                self.tag_candidates = enumerate_side(
                    self._config(True), self._params, 0.125
                )[0]
        elif self.config.has_tag:
            raise ValueError(
                "a fixed partition is only supported for a single array"
            )
        self._built = True
        return self

    def _array(self, partition, params, tag):
        return build_array(
            "CACTI",
            self._config(tag, rebuild=True),
            params,
            partition.integers,
            (partition.mat_width, partition.mat_height),
            device_ty=self.config.device,
            core_ooo=self.technology.core_ooo,
        )

    def search(self):
        if self.result is not None:
            return self.result
        self.build()
        options = self.search_options
        timing = None
        if options.partition is None:
            res = optimize_array_partition(
                self._config(),
                self._params,
                tag_cfg=self._config(True) if self.config.has_tag else None,
                opt_for_clk=options.opt_for_clk,
                opt_local=options.opt_local,
                throughput=options.throughput,
                latency=options.latency,
            )

            def part(candidate):
                return Partition(
                    candidate.Nspd,
                    candidate.Ndwl,
                    candidate.Ndbl,
                    candidate.Ndcm,
                    candidate.Ndsam1,
                    candidate.Ndsam2,
                    candidate.uca.mat.area_w,
                    candidate.uca.mat.area_h,
                )

            partitions = {"Data": part(res.winner.data)}
            if self.config.has_tag:
                partitions["Tag"] = part(res.winner.tag)
            checked = hasattr(res, "satisfied")
            timing = dict(
                checked=checked,
                engaged=checked and res.engaged,
                throughput_ok=res.throughput_ok if checked else True,
                latency_ok=res.latency_ok if checked else True,
                access_time=res.winner.access_time,
                cycle_time=res.winner.cycle_time,
            )
        else:
            partitions = {"Data": options.partition}
        self.candidates = ()
        self.tag_candidates = ()
        built = {
            name: self._array(
                p, self._params, name == "Tag" or self.config.is_tag
            )
            for name, p in partitions.items()
        }

        def coefficients(array):
            return array if options.scaling == "mcpat" else array.uca

        data = coefficients(built["Data"])
        energies = {"Read": data.read, "Write": data.write}
        if self.config.associativity == 0:
            energies["Search"] = data.search
        if "Tag" in built:
            tag = coefficients(built["Tag"])
            energies.update(TagRead=tag.read, TagWrite=tag.write)

        def static_at(t):
            params = self.technology.parameters(t)
            rows = {}
            for name, partition in partitions.items():
                wrapped = self._array(
                    partition, params, name == "Tag" or self.config.is_tag
                )
                array = coefficients(wrapped)
                sub = (
                    wrapped.longer_channel_leakage
                    if options.scaling == "mcpat"
                    and self.technology.longer_channel
                    else array.leakage
                )
                rows[name] = {
                    "leakage": sub,
                    "gate_leakage": array.gate_leakage,
                }
            return rows

        static = StaticCoefficients(
            static_at,
            partitions=partitions,
            timing=timing,
            resolved_config={
                "array": asdict(self.config),
                "technology": asdict(self.technology),
                "search": asdict(options),
            },
        )
        self.result = ArrayResult(energies, static, partitions, timing)
        return self.result


@lru_cache(maxsize=32)
def _derive(config, technology, search_options):
    return (
        CactiArray(
            config, technology=technology, search_options=search_options
        )
        .build()
        .search()
    )
