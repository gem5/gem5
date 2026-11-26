# SPDX-License-Identifier: BSD-3-Clause
"""Public standalone CACTI interface; no gem5 dependency."""

from copy import deepcopy

from .cacti_array import (
    ArrayResult,
    CactiArray,
    _derive,
)
from .cacti_cache import derive_cache
from .gem5_inputs import (
    derive_buffer_counts,
    derive_cache_config,
)
from .mcpat_solver.records import (
    ArrayConfig,
    Partition,
    SearchOptions,
    Technology,
)


def derive_cacti_coefficients(
    cache_obj=None, *, array_config=None, options=None
):
    """Return (joules per operation, temperature-dependent leakage in watts)."""
    options = dict(options or {})
    technology = options.pop("technology", Technology())
    search = options.pop("search_options", SearchOptions())
    buffer_counts = options.pop("buffer_counts", None)
    cache_options = {
        k: options.pop(k)
        for k in ("buffer_policy", "write_policy", "address_width")
        if k in options
    }
    if cache_obj is not None:
        if array_config is not None:
            raise ValueError("provide a cache object or explicit geometry")

        if buffer_counts is None:
            buffer_counts = derive_buffer_counts(cache_obj)
        array_config = derive_cache_config(
            cache_obj,
            address_width=cache_options.get("address_width", 32),
            **options,
        )
    elif options:
        raise ValueError(f"unknown options: {sorted(options)}")
    if array_config is None:
        raise ValueError("cache_obj or array_config is required")
    if isinstance(array_config, dict):
        array_config = ArrayConfig(**array_config)
    if buffer_counts is not None:

        energy, static = derive_cache(
            array_config,
            technology,
            search,
            tuple(sorted(buffer_counts.items())),
            tuple(sorted(cache_options.items())),
        )
        return deepcopy(energy), static
    if set(cache_options) - {"address_width"}:
        raise ValueError("buffer/write policy requires buffer_counts")
    result = _derive(array_config, technology, search)
    return dict(result.activation_energies), result.static_coeffs
