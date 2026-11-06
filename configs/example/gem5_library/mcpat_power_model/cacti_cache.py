# SPDX-License-Identifier: BSD-3-Clause
"""Standalone cache/buffer coefficients and the runtime compatibility view."""

from functools import lru_cache
from types import MappingProxyType

from .cacti_array import CactiArray
from .coefficients import StaticCoefficients
from .mcpat_solver.cache_buffers import build_cache_buffers
from .mcpat_solver.records import (
    ArrayConfig,
    SearchOptions,
    Technology,
)


class CactiCache:
    def __init__(
        self,
        config,
        *,
        technology=None,
        search_options=None,
        entry_counts=None,
        buffer_policy="configured",
        write_policy="write_back",
        address_width=32,
        level="l1d",
        prefix="dcache",
        bucket="Data Cache",
    ):
        if not config.has_tag:
            raise ValueError(
                "cache composition requires paired data/tag geometry"
            )
        self.config = config
        self.technology = technology or Technology()
        self.search_options = search_options or SearchOptions(scaling="mcpat")
        self.entry_counts = MappingProxyType(
            dict(entry_counts or dict(mshr=0, fill=0, prefetch=0, wbb=0))
        )
        if set(self.entry_counts) != {
            "mshr",
            "fill",
            "prefetch",
            "wbb",
        } or any(
            not isinstance(v, int) or isinstance(v, bool) or v < 0
            for v in self.entry_counts.values()
        ):
            raise ValueError(
                "buffer_counts requires four nonnegative integer entry counts"
            )
        if buffer_policy not in (
            "configured",
            "mcpat",
        ) or write_policy not in ("write_back", "write_through"):
            raise ValueError("unsupported buffer or write policy")
        self.buffer_policy, self.write_policy = buffer_policy, write_policy
        self.address_width, self.level, self.prefix = (
            address_width,
            level,
            prefix,
        )
        self.bucket = bucket
        self.result = None

    def derive(self):
        if self.result is not None:
            return self.result
        c, t = self.config, self.technology
        main = (
            CactiArray(c, technology=t, search_options=self.search_options)
            .build()
            .search()
        )
        components = dict(Data=main)
        energies = dict(main.activation_energies)
        geometry = build_cache_buffers(
            level=self.level,
            prefix=self.prefix,
            capacity=c.capacity,
            line_bytes=c.line_bytes,
            assoc=c.associativity,
            phy_addr_width=self.address_width,
            entry_counts=self.entry_counts,
            cache_policy=self.write_policy,
            fetch_or_memory_ports=max(c.rw_ports, c.read_ports, c.write_ports),
            partitions=None,
            buffer_policy=self.buffer_policy,
            wire_is_mat_type=(
                c.wire_inside
                if c.wire_inside is not None
                else (0 if t.embedded else 2)
            ),
            wire_os_mat_type=c.wire_outside,
            wt_overhead=(
                c.wire_overhead
                if c.wire_overhead is not None
                else (30 if t.embedded else 0)
            ),
        )
        for name, spec in geometry.items():
            g = spec._cfg_kwargs
            cfg = ArrayConfig(
                capacity=g["capacity"],
                line_bytes=g["block_sz"],
                data_width=g["out_w"],
                tag_width=g["tag_w"],
                associativity=0,
                rw_ports=g["num_rw_ports"],
                search_ports=g["num_search_ports"],
                wire_inside=g["wire_is_mat_type"],
                wire_outside=g["wire_os_mat_type"],
                wire_overhead=g["wt_overhead"],
                device=spec._device_ty,
            )
            result = (
                CactiArray(
                    cfg, technology=t, search_options=self.search_options
                )
                .build()
                .search()
            )
            components[name] = result
            energies[name] = dict(result.activation_energies)

        def at_sample(temperature):
            rows = dict(main.static_coeffs.per_component(temperature))
            for name, result in components.items():
                if name != "Data":
                    rows[name] = result.static_coeffs.per_component(
                        temperature
                    )["Data"]
            return rows

        static = StaticCoefficients(
            at_sample,
            partitions={
                name: result.partitions for name, result in components.items()
            },
            timing={
                name: result.timing for name, result in components.items()
            },
            resolved_config=dict(
                array=main.static_coeffs.resolved_config,
                buffers=dict(self.entry_counts),
                buffer_policy=self.buffer_policy,
                write_policy=self.write_policy,
                address_width=self.address_width,
            ),
        )
        static.model = self
        self.result = energies, static
        return self.result

    def cache_activation_energies(self):
        return {self.bucket: self.derive()[0]}

    def leakage_by_unit_at(self, bucket, temperature):
        if bucket != self.bucket:
            raise KeyError(bucket)
        return self.derive()[1].per_component(temperature)

    def leakage_at(self, bucket, temperature):
        return sum(
            v["leakage"]
            for v in self.leakage_by_unit_at(bucket, temperature).values()
        )

    def gate_leakage_at(self, bucket, temperature):
        return sum(
            v["gate_leakage"]
            for v in self.leakage_by_unit_at(bucket, temperature).values()
        )


@lru_cache(maxsize=32)
def derive_cache(config, technology, search_options, counts, cache_options):
    """Reuse independent standalone searches by immutable resolved inputs."""
    return CactiCache(
        config,
        technology=technology,
        search_options=search_options,
        entry_counts=dict(counts),
        **dict(cache_options),
    ).derive()
