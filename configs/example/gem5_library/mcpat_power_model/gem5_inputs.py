# SPDX-License-Identifier: BSD-3-Clause
"""Duck-typed cache and clock inputs, without importing gem5 or composition."""

import math

from .mcpat_solver.records import ArrayConfig


def _value(value):
    return getattr(value, "value", value)


def derive_cache_config(
    cache,
    *,
    board=None,
    line_bytes=None,
    address_width=32,
    tag_width=None,
    banks=1,
    **changes,
):
    if line_bytes is None:
        if board is None:
            raise ValueError("a board or explicit line_bytes is required")
        line_bytes = int(board.get_cache_line_size())
    capacity, assoc = int(_value(cache.size)), int(cache.assoc)
    if tag_width is None:
        sets = capacity / (line_bytes * assoc)
        if sets <= 0:
            raise ValueError("cache has no sets")
        tag_width = (
            address_width
            - math.ceil(math.log2(sets))
            - math.ceil(math.log2(line_bytes))
            + 5
        )
    config = ArrayConfig(
        capacity=capacity,
        line_bytes=line_bytes,
        data_width=line_bytes * 8,
        associativity=assoc,
        tag_width=tag_width,
        banks=banks,
        has_tag=True,
        **changes,
    )
    config.validate()
    return config


def derive_buffer_counts(cache):
    prefetcher = getattr(cache, "prefetcher", None)
    counts = dict(
        mshr=int(cache.mshrs),
        fill=int(cache.mshrs),
        prefetch=(
            int(prefetcher.queue_size)
            if prefetcher and hasattr(prefetcher, "queue_size")
            else 0
        ),
        wbb=int(cache.write_buffers),
    )
    if any(value < 0 for value in counts.values()):
        raise ValueError("buffer entry counts must be nonnegative")
    return counts


def cpu_clock_mhz(board, cpu):
    def resolve(value, owner):
        if type(value).__module__.startswith("m5.proxy"):
            return value.unproxy(owner)
        return value

    domain = resolve(getattr(cpu, "clk_domain", None), cpu)
    if domain is None:
        domain = board.get_clock_domain()
    divider, seen = 1, set()
    while not hasattr(domain, "clock"):
        if id(domain) in seen:
            raise ValueError("cyclic CPU clock domains")
        seen.add(id(domain))
        divider *= int(_value(domain.clk_divider))
        domain = resolve(domain.clk_domain, domain)
    level = int(_value(getattr(domain, "init_perf_level", 0)))
    period = float(_value(domain.clock[level])) * divider
    if not math.isfinite(period) or period <= 0:
        raise ValueError("CPU clock period must be finite and positive")
    return int(round(1e-6 / period))
