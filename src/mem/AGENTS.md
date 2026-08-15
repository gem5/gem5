# Memory System Agent Notes

This directory contains gem5's memory-system infrastructure: ports, packets,
requests, crossbars, bridges, classic caches, memory controllers, protocol
interfaces, external memory wrappers, probes, and Ruby.

## Request Paths

Start memory investigations by identifying the configured `System.mem_mode`
(`atomic`, `timing`, or `atomic_noncaching`), the cache hierarchy (Classic or
Ruby), and the request origin and path (CPU-side, memory-side, DMA, or external
port). `Request` carries architectural request metadata, `Packet` carries a
specific memory transaction, and ports define how SimObjects exchange packets.

Do not assume timing-mode behavior applies to atomic or functional access.
Functional accesses are zero-time debug operations for inspecting or updating
simulated state and are not a `System.mem_mode`. Atomic accesses flow through
the configured memory system without event-driven timing, while
`atomic_noncaching` is used for paths that bypass caches, including KVM and
Ruby atomic accesses. Timing accesses model queued, asynchronous packet flow.
Snoops are cache-coherence traffic within these access mechanisms, not a
separate memory mode.

## Classic, Ruby, And Controllers

Classic caches and interconnects are mostly hand-written C++ SimObjects under
`src/mem/cache` and related files. Ruby coherence protocols live under
`src/mem/ruby` and are generated from SLICC inputs. A change to packet fields,
request flags, port semantics, or memory-controller behavior can affect both
Classic and Ruby even when the immediate bug appears in only one hierarchy.

For memory-controller work, check the relevant Python SimObject declaration,
C++ interface, QoS behavior, external wrapper, and config/test coverage. Keep
DRAM interface changes aligned with existing controllers and third-party
wrapper expectations.

## Validation

Prefer focused builds and the narrowest unit test or simulation that exercises
the touched path before broad validation. Compilation alone does not validate
packet ordering or coherence behavior. When reproducing a memory-system bug,
preserve the exact CPU model, memory mode, cache hierarchy, coherence protocol,
and workload because small configuration differences can route requests
through different code paths.
