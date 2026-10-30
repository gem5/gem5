# SALAM Object Overview

SALAM represents accelerators with gem5 SimObjects under `src/salam`.
Software controls devices through MMIO; accelerator ports access system
memory, local storage, and streams.

## AccCluster

`AccCluster` groups accelerators, local memories, DMAs, and buses. Its
local bus connects devices inside the cluster; its coherency bus connects
to the system. Workload YAML assigns devices within
`[0x2f000000, 0x30000000)` on the supported ARM platform.

## CommInterface

`CommInterface` exposes the accelerator's MMIO registers and memory ports.
Its register map contains flags for launch/status, a reserved configuration
region, and arguments captured when the accelerator starts.

| Port | Purpose |
|---|---|
| PIO | Program and inspect control registers |
| Local | Access devices on the local cluster bus |
| ACP | Access memory outside the cluster |
| Stream | Transfer data when a producer or consumer is ready |
| SPM | Access scratchpads with readiness controls |

## AccComputeUnit and LLVMInterface

`AccComputeUnit` is the base timed compute engine. `LLVMInterface` reads
LLVM IR and executes its control and dataflow graph using runtime arguments
from `CommInterface`. It issues compute and memory operations and tracks
their completion.

## HWInterface

`HWInterface` supplies instruction-to-functional-unit mappings, functional
unit inventory, and instruction cycle counts. Each accelerator has its own
units. See [functional unit scheduling](README_SALAM.md#functional-unit-scheduling)
for allocation and admission settings.

## SALAMPowerModel

`HWInterface.salam_power_model` observes functional-unit inventory,
operation admissions, and a static SSA-storage approximation.
Each scratchpad and register bank has a separate `ppa` child that observes
serviced accesses, including CPU and DMA traffic. Shared memory is counted
once.

The model reports known area, energy, power, and separate coverage flags.
Missing coefficients remain unavailable. See
[Statistics](Building_and_Integrating_Accelerators.md#statistics) for run
outputs.

## ScratchpadMemory

`ScratchpadMemory` provides accelerator-local storage with optional access
synchronization. With `ReadyMode`, accesses depend on each location's
readiness. Workload defaults are `ReadyMode=false`, `ResetOnRead=true`,
`ReadOnInvalid=false`, and `WriteOnValid=true`. Use YAML booleans for these
fields.

## RegisterBank

`RegisterBank` stores accelerator-local scalar values, pointers, or arrays.
Reads sample visible storage when accepted. Writes become visible after
their deferred write batch is published; a completed write response
guarantees that subsequent reads see the stored value.

## NoncoherentDma

`NoncoherentDma` copies data between system memory and local storage through
two FIFOs. The MMIO layout is in `src/salam/noncoherent_dma.hh`. See
[DMA configuration](README_SALAM.md#dma-configuration) for buffer and request
limits.

## StreamBuffer, StreamDma, and stream ports

`StreamBuffer` is a FIFO for producer/consumer connections.
`StreamDma` connects these streams to system memory and supports repeated
frames. Its MMIO layout is in `src/salam/stream_dma.hh`.

Stream ports transfer only when data or space is available. Producers and
consumers retry after backpressure, without treating availability checks as
data transfers.

## Timing model

The workload's Compute clock drives `LLVMInterface` and `CommInterface`;
its Fabric clock drives local buses, DMAs, streams, and local memories.
The coherency bus and cluster cache use the system clock. See the
[clock settings](Building_and_Integrating_Accelerators.md#set-the-cluster-clocks)
for parameter units and defaults.

- A launch starts on the compute edge after the edge on or following the
  host's run-bit write. A fixed-latency operation issued on edge k completes
  on edge k + L. Loads and stores complete from their memory responses.
- `CommInterface` services outstanding requests every `process_cycles`
  compute cycles. A request issued while idle waits that interval.
  Memory service precedes compute on the same edge; the next compute pass
  sees responses that have arrived.
- Scratchpad responses become ready `access_cycles` fabric cycles after
  the first fabric edge at or after arrival, plus a uniformly sampled
  extra of up to `access_cycles_var` cycles. Response backpressure can delay
  other ports sharing the queue.
- Register banks publish writes in batches after `write_cycles` fabric
  cycles from the first write's edge. Each response waits for both its
  own `access_cycles` delay and publication of its batch. Private-port
  reads complete immediately; load-port reads wait `access_cycles`.
  Reads accepted before publication may sample the old value.
- PIO delays remain time values rather than cycle counts.

The `SALAMLaunch` debug flag reports launch/completion ticks and cycles.

## Execution semantics

A module containing one function uses it as the entry function. With
multiple functions, the workload launcher uses the function named `top`.

Supported terminators `br`, `ret`, and `switch_inst` have no separate
modeled latency; their cycle counts must be zero. Other terminators,
including `indirectbr`, `invoke`, `resume`, and `unreachable`, are rejected.
Unconditional branch chains are followed eagerly; a chain revisiting a
block is rejected. Conditional loops follow their runtime branches.

Calls complete on the caller's compute pass after the callee returns,
plus their configured call latency in further passes. A call latency of N
therefore completes N+1 compute edges after the return.

Volatile loads/stores wait for earlier volatile accesses, even at disjoint
addresses. Calls wait for earlier memory accesses, and younger accesses
wait for the call to return. Ordinary accesses retain address-overlap
dependencies. Volatile accesses do not provide atomic synchronization
between independent requestors.

Pointer and GEP index widths must be equal in each address space used by
the accelerator graph. Unequal widths are rejected before execution.

## Statistics

Counters in `stats.txt` cover the current reset window. A reset clears
counters while preserving device state, functional-unit occupancy, buffers,
and queued work. A response after reset belongs to the new window even if
its request was accepted before reset.

The printed kernel summary covers the full invocation. `Compute passes`
counts scheduler passes, including the first and final pass.
`Invocation duration` and `Runtime` measure elapsed simulator time from
accepted launch to completion in ticks and microseconds. Consecutive
passes include both endpoints: 14 passes span 13 clock periods.

Names below are relative to each object's path in `config.ini`:

- `SALAMPowerModel`: `knownAreaUm2`, `knownDynamicEnergyJ`,
  `knownLeakageEnergyJ`, `knownTotalEnergyJ`, `elapsedSeconds`, and
  `knownAveragePowerW` describe known subtotals. Independent coverage flags
  distinguish those subtotals from complete modeled estimates. FU/SSA
  invocation summaries remain separate from memory statistics windows.
- `LLVMInterface`: `invocations`, `cycles`, `issued` and `completed` by
  operation class, `opcodesIssued`, `fuDenials`, and `memoryOrderWaits`.
  `reservedOps`, `computeOps`, `readOps`, and `writeOps` are time-weighted
  occupancies.
- `LLVMInterface.cycleCategories`: Each compute pass belongs to one category.
  Priority is `issue`, `fuStall`, `memoryWait`, `computeWait`, then `other`.
  These count passes, not simultaneous causes.
- `HWInterface.<unit alias>`: `instances` is current inventory; `admissions`
  and `denials` count scheduling decisions. `inFlight` and `busyInstances` are
  time-weighted averages. `utilization` is busy-instance time divided by
  instance count and window ticks, or zero with no instances.
- `CommInterface`: `starts`, `finishes`, logical `requests`, send `attempts`,
  `accepted` packets, `responses`, and response `bytes` distinguish operations
  from retries and completion.
- `NoncoherentDma`: `transfers` and `bytes` count starts and requested bytes;
  `beats` counts FIFO-to-FIFO moves. `writePackets` counts
  write FIFO requests, which may split at cache-line boundaries. `active` is
  the time-weighted fraction with a transfer in progress.
- DMA `memoryPackets` / `clusterPackets`: `readAttempts` / `writeAttempts`
  include retries; `readPackets` / `writePackets` count downstream acceptance;
  `readResponses` / `writeResponses` and `readBytes` / `writeBytes` count
  completion. `clusterPackets` exists on `NoncoherentDma` for its
  accelerator-side port. Direct atomic backdoor copies are not packets.
- `StreamDma.transfers`: `readTransfers` / `writeTransfers` count starts;
  `readFrames` / `writeFrames` count completed frames; `readBytes` /
  `writeBytes` count requested frame bytes. `readActive` / `writeActive` are
  time-weighted active fractions. Read means memory-to-stream; write means
  stream-to-memory.
- StreamBuffer / StreamDma `stream`: `readAttempts` / `writeAttempts` count
  timing stream-port sends, including retries. `readPackets` / `writePackets`
  count accepted beats; `readBytes` / `writeBytes` count FIFO data movement.
  Availability polls are not attempts. Legacy StreamDma MMIO buffer accesses
  count accepted data movement but are not stream-port attempts.
- `StreamBuffer`: `bufferOccupancy` is time-weighted buffered bytes; it retains
  the actual occupancy across a statistics reset.

`memoryOrderWaits` counts scheduling observations of ready loads or stores
blocked by older memory accesses. It includes the observation that creates
an in-flight RAW dependency; it does not measure elapsed wait cycles or
count each dependency repeatedly while that instruction is unready.

The average statistics retain current state at a reset while beginning a
new integration interval. Multiplying an active fraction by its statistics
window gives active time, subject to the text output's decimal precision.
Packet counts can exceed logical operations because a transfer can split
into multiple memory packets. Attempts can exceed accepted packets because
of retries. Repeated dumps do not clear counters; use a reset to start a
new window.

For instruction issue tracing, add `--debug-flags=SALAMIssue` before the
configuration script on the gem5 command line. Tracing observes the same
execution and accounting; it does not change scheduling.
