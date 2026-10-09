# SPDX-License-Identifier: BSD-3-Clause
"""Resolve SimObject inputs at the gem5 boundary; numerical code stays pure."""

from copy import deepcopy
from types import SimpleNamespace

from .coefficients import StaticCoefficients
from .gem5_inputs import (
    cpu_clock_mhz,
    derive_buffer_counts,
    derive_cache_config,
)
from .mcpat_solver_bridge_live import build_machine_model_from_config


def discover_caches(hierarchy, index):
    if not all(
        hasattr(hierarchy, name)
        for name in ("l1icaches", "l1dcaches", "l2cache")
    ):
        raise ValueError(
            "unsupported hierarchy: provide explicit cache objects "
            "or complete cache_configs and buffer sizes"
        )
    try:
        caches = dict(
            l1icache=hierarchy.l1icaches[index],
            l1dcache=hierarchy.l1dcaches[index],
            l2cache=hierarchy.l2cache,
        )
    except IndexError as error:
        raise ValueError(
            "hierarchy lacks caches for the selected CPU"
        ) from error
    if any(value is None for value in caches.values()):
        raise ValueError(
            "explicit configuration is required for absent caches"
        )
    return caches


def model_coefficients(model):
    buckets = ("Core", "Instruction Cache", "Data Cache", "L2")
    partitions, arrays = {}, {}

    def visit(path, obj):
        if hasattr(obj, "_dp_kwargs"):
            partitions[path] = dict(obj._dp_kwargs)
            arrays[path] = dict(
                geometry=obj._cfg_kwargs,
                partition=obj._dp_kwargs,
                device=obj._device_ty,
                core_ooo=obj._core_ooo,
            )
        elif isinstance(obj, dict):
            for name, value in obj.items():
                visit(f"{path}/{name}", value)
        elif hasattr(obj, "__dict__"):
            for name in ("data_array", "tag_array", "buffers", "_arrays"):
                if hasattr(obj, name):
                    visit(f"{path}/{name}", getattr(obj, name))

    for name in (
        "_core_arrays",
        "_icache_cache",
        "_dcache_cache",
        "_l2_cache",
        "_btb_cache",
        "_itlb",
        "_dtlb",
    ):
        visit(name, getattr(model, name))

    def at_sample(temperature):
        return {
            f"{bucket}/{name}": values
            for bucket in buckets
            for name, values in model.leakage_by_unit_at(
                bucket, temperature
            ).items()
        }

    static = StaticCoefficients(
        at_sample,
        partitions=partitions,
        timing=getattr(model, "timing_report", []),
        resolved_config=dict(
            machine=model.machine_params,
            core=dict(model._core_params),
            arrays=arrays,
            composition=getattr(model, "resolved_config", {}),
        ),
    )
    static.model = model
    return model.activation_energies(), static


def derive_coefficients(
    board, cpu=None, cache_hierarchy=None, *, options=None
):

    options = dict(options or {})
    policy = options.pop("buffer_policy", "configured")
    hierarchy = cache_hierarchy or board.get_cache_hierarchy()
    cores = board.get_processor().get_cores()
    selected = (
        list(enumerate(cores))
        if cpu is None
        else [(i, c) for i, c in enumerate(cores) if c is cpu or c.core is cpu]
    )
    if not selected:
        raise ValueError("CPU does not belong to this board")
    models, energy, statics = [], {}, []
    explicit = options.pop("cache_objects", None)
    for index, core in selected:
        cpu_type = str(core.get_type().value).lower()
        if cpu_type not in ("timing", "minor", "o3"):
            raise ValueError(f"unsupported CPU type: {cpu_type}")
        if explicit is not None:
            caches = dict(explicit)
            if set(caches) != {"l1icache", "l1dcache", "l2cache"} or any(
                v is None for v in caches.values()
            ):
                raise ValueError(
                    "cache_objects requires all modeled cache roles"
                )
        elif "cache_configs" in options and all(
            options["cache_configs"].get(key) is not None
            for key in (
                "icache_config",
                "dcache_config",
                "L2_config",
                "icache_buffer_sizes",
                "dcache_buffer_sizes",
                "L2_buffer_sizes",
            )
        ):
            caches = {}
        else:
            caches = discover_caches(hierarchy, index)
        processor = SimpleNamespace(
            get_cores=lambda: [core], get_num_cores=lambda: 1
        )

        class BoardView:
            def get_processor(self):
                return processor

            def __getattr__(self, name):
                return getattr(board, name)

        local_options = deepcopy(options)
        overrides = local_options.setdefault("overrides", {})
        core_overrides = overrides.setdefault("core", {})
        if cpu_type == "o3":
            core_overrides.setdefault(
                "inst_buffer_size", int(core.core.fetchQueueSize)
            )
        elif cpu_type == "minor":
            core_overrides.setdefault(
                "store_buffer_sz", int(core.core.executeLSQStoreBufferSize)
            )
        model = build_machine_model_from_config(
            BoardView(),
            cpu_type,
            **caches,
            **local_options,
            buffer_policy=policy,
            reuse=True,
        )
        energies, static = model_coefficients(model)
        models.append(model)
        statics.append(static)
        if len(selected) == 1:
            return energies, static
        energy[f"CPU{index}"] = {
            k: v for k, v in energies.items() if k != "L2"
        }
    energy["L2"] = models[0].cache_activation_energies()["L2"]

    def at_sample(t):
        rows = {}
        for (index, _), static in zip(selected, statics):
            rows.update(
                {
                    f"CPU{index}/{name}": v
                    for name, v in static.per_component(t).items()
                    if not name.startswith("L2/")
                }
            )
        rows.update(
            {
                name: v
                for name, v in statics[0].per_component(t).items()
                if name.startswith("L2/")
            }
        )
        return rows

    partitions = {}
    timing = []
    for (index, _), item in zip(selected, statics):
        for path, partition in item.partitions.items():
            if path.startswith("_l2_cache/"):
                if index == selected[0][0]:
                    partitions[path] = partition
            else:
                partitions[f"CPU{index}/{path}"] = partition
        timing.append(dict(cpu=index, diagnostics=item.timing))
    static = StaticCoefficients(
        at_sample,
        partitions=partitions,
        timing=timing,
        resolved_config={
            f"CPU{index}": s.resolved_config
            for (index, _), s in zip(selected, statics)
        },
    )
    static.models = models
    return energy, static
