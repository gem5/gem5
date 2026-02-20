# SALAM Object Overview

All source code for these objects are stored in **src/salam**.

At a high level, SALAM models an accelerator as a set of gem5 SimObjects with a **control plane** (software-visible MMRs over PIO) and a **data plane** (ports used to access system memory, accelerator-local memories, and optional streaming channels). A typical setup wires one or more accelerators (compute engines) to a **CommInterface** for programmability and external memory access, optionally organized inside an **AccCluster** along with scratchpads, DMAs, and stream components.

## AccCluster

This is an optional simulation object useful for organizing accelerators and resources shared between accelerators. AccCluster provides utilities for connecting accelerators and other shared resources into the larger system.

Conceptually, AccCluster is where you define "what is local to the accelerator subsystem" (local bus, scratchpads, stream buffers, register banks, local DMA endpoints) and "what connects to the rest of the SoC" (system/coherent side connections). If you have multiple accelerators and shared local resources, AccCluster is the place to wire them consistently.

## CommInterface

The communications interface is the base gem5 component of an accelerator in gem5-SALAM. It provides programmability as well as system memory access to the accelerator. It also provides mechanisms for synchronization, including memory interrupt lines.

CommInterface is the software-visible control point: the CPU programs it via MMIO/PIO, it stores runtime configuration and variable addresses, and it exposes the ports that the accelerator datapath uses to interact with memory and local devices. In most SALAM systems, CommInterface is the "hub" that sits between software control and the datapath execution engine.

The memory map for the CommInterface object is broken into three sections:

* Flags: Provides runtime status information and switches for invoking the interface
* Config: Currently has no function, but is reserved for a future version
* Variables: Addresses for runtime variables or values that will be pulled upon invocation.

### Ports:

* PIO: Connects to MMRs and provides external devices the ability to program the CommInterface.

* Local Ports: Provides access to other devices within an accelerator's local cluster.

* ACP Ports: Provides access to devices outside of the accelerator's local cluster.

* Stream Ports: Implements an AXI-stream like paradigm that limits read and write to data availability. It can be used in producer-consumer schemes with other devices using StreamBuffers or StreamDMAs.

* SPM Ports: Provides access to scratchpads using the additional synchronization controls provided by the scratchpad memory.

## AccComputeUnit

AccComputeUnit is the base compute object used for accelerator datapaths in SALAM. It defines the common structure for a timed, event-driven compute engine (i.e., a “tick” that advances accelerator execution) and provides the hooks used by derived datapath models.

In practice, you generally interact with the derived datapath object (LLVMInterface), but AccComputeUnit is useful to understand because it defines the shared notion of “accelerator execution advancing over time” within gem5.

## LLVMInterface

The LLVM Interface represents the data path of the accelerator. It is what parses the LLVM IR file to generate the hardware data path and then generates and executes the LLVM Control and Data Flow Graph (CDFG) using runtime data provided by the CommInterface.

LLVMInterface is the execution engine: it drives the accelerator’s control/dataflow progression, initiates memory operations, and models completion and synchronization at the accelerator level. It relies on (1) the LLVM IR parsing support (LLVMRead) to interpret the IR and (2) the hardware modeling configuration (via HWInterface / HWModeling) to decide how instructions map to modeled hardware resources (functional units, cycle counts, opcode behaviors, etc.).

## HWInterface

HWInterface is the configuration and modeling interface between the LLVM-driven execution engine and the underlying hardware model used for timing and (optionally) power/statistics.

It aggregates the modeled hardware description components used by the datapath, such as:

* functional unit inventory and properties,
* instruction configuration (mapping instructions to resources and costs),
* opcode mappings,
* cycle count models and simulator configuration knobs,
* optional power/statistics hooks where enabled.

This object is typically instantiated as part of an accelerator definition and passed to the compute engine (LLVMInterface) so execution reflects the intended microarchitectural model.

## ScratchpadMemory

ScratchpadMemory is a custom fast-access memory for accelerators. It includes access synchronization mechanisms such as ready mode. When access synchronization is activated accelerators will not be able to access data that is not marked ready. Furthermore, additional controls will be placed on reads and writes to the scratchpad in order to implement various sync mechanisms.

ScratchpadMemory is usually attached on the local side of an accelerator subsystem (often via AccCluster) and accessed via the CommInterface SPM ports. It is the standard way to model accelerator-local SRAM with explicit movement (typically through DMA) rather than implicit cache behavior.

## RegisterBank

RegisterBank is a small, accelerator-local storage object used for register-like state and runtime variables. It is commonly used to hold scalar configuration values, pointers, or small arrays that are convenient to access as local state rather than through system memory.

RegisterBank is typically mapped into an address range and connected within the local accelerator subsystem, so software or the accelerator can read/write it depending on how the system is wired.

## NoncoherentDma

NoncohherentDma provides a memory-to-memory transfer. This is useful for copying data to and from system memory and scratchpads. The MMR layout of the NoncoherentDma is described in noncoherent_dma.hh.

NoncoherentDma is generally used when you want explicit bulk movement (e.g., system memory → scratchpad before compute, scratchpad → system memory after compute) without stream semantics. It is programmed through its MMRs and performs timed transfers through the memory system based on the connectivity you provide (system-side and accelerator-local side).

## StreamBuffer

StreamBuffer is a small FIFO buffer that enables AXI-Stream like communication between devices.

StreamBuffer is most useful when modeling producer/consumer datapaths that should naturally backpressure: a producer can only push when space is available, and a consumer can only pop when data is available. StreamBuffers are typically paired with stream ports and/or a StreamDma to bridge streaming and memory-mapped domains.

## StreamDma

The StreamDma provides DMA access between traditional memory objects, scratchpads, and an AXI-stream like interface. It also supports auto-play features commonly found in video DMAs. The memory map is defined in stream_dma.hh.

Conceptually, StreamDma is the “bridge” between memory-mapped buffers (system memory or scratchpads) and streaming endpoints. It is useful when you want streaming behavior at the boundary of an accelerator, but the rest of the system still uses conventional addressable memory.

## Stream Ports

Stream ports implement the availability-gated transfer semantics used by StreamBuffer and StreamDma connections. They enforce the "only transfer when data is available / space is available" behavior that distinguishes the stream path from normal memory ports.

In SALAM, stream ports are exposed by the CommInterface so an accelerator datapath can participate in streaming producer/consumer patterns without treating streams as ordinary load/store memory accesses.
