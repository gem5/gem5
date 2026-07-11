# Memory System Agent Notes

This directory contains gem5's memory-system infrastructure: ports, packets,
requests, crossbars, bridges, classic caches, memory controllers, protocol
interfaces, external memory wrappers, probes, and Ruby.

## Request Paths

Start memory investigations by identifying the mode and path: atomic, timing,
functional, or snoop; Classic or Ruby; CPU-side, memory-side, DMA, or external
port. `Request` carries architectural request metadata, `Packet` carries a
specific memory transaction, and ports define how SimObjects exchange packets.

Do not assume timing-mode behavior applies to atomic or functional access.
Functional accesses are for inspecting or updating simulated state, atomic
accesses model instantaneous service with latency accounting, and timing
accesses model queued/asynchronous packet flow.

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

Prefer focused builds or unit tests for touched components before broad
simulator builds. When reproducing memory-system bugs, preserve the exact CPU
model, memory mode, cache hierarchy, coherence protocol, and workload because
small config differences can route requests through different code paths.
