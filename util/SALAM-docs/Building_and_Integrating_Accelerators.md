# Building and Integrating Accelerators in gem5-SALAM

This guide uses the in-tree BFS example to show, step by step, how to define, build, and run a SALAM accelerator in gem5.

SALAM models an accelerator from a functional description of its kernel. The kernel is written in a high-level language such as C, compiled to LLVM IR, and parsed into a control and dataflow graph (CDFG). `LLVMInterface` then executes that CDFG as an event-driven, timed model inside gem5. In the full system, gem5 runs a normal CPU program (an ELF binary). That host program configures and launches one or more SALAM accelerators through MMIO.

The in-tree example is breadth-first search (BFS). BFS labels each graph node with its distance (level) from a starting node. In this workload, the traversal runs as an accelerator kernel over accelerator-local node, edge, and level storage.

SALAM supports hardware–software codesign by letting the user specify both the accelerator’s function and the hardware it runs on.

On the software (workload) side, the user provides:

* a host program (`sw/main.elf`) that runs on the simulated CPU and drives the accelerator over MMIO,
* one or more accelerator kernels compiled to LLVM IR (`hw/*.ll`),
* a YAML specification (`config.yml`) that describes the accelerator cluster topology and instruction timing,
* an auto-generated header (`*_hw_defines.h`) that keeps software-visible MMIO addresses consistent with gem5’s address map.

The hardware profile defines functional units and instruction mappings;
the workload YAML sets instruction timing.

Together, these parts let users model and explore accelerator microarchitecture and system integration early, without writing RTL.

## Step 1: Set Up the Workload

The BFS example is located under:

**configs/example/gem5_library/salam-benchmarks/src/bfs**

Set the repository and workload paths before building:

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src
cd $M5_PATH
```

The required packages, LLVM versions, and ARM cross-compiler are listed in **util/SALAM-docs/README_SALAM.md**.

The BFS directory contains the following main files and directories:

* `hw/bfs.c`: the BFS compute kernel,
* `hw/top.c`: the controller kernel,
* `sw/main.cpp`: the host program,
* `config.yml`: the accelerator cluster and instruction timing description,
* `Makefile`: the top-level workload build file.

The BFS accelerator is divided into two kernels. `bfs.c` performs the graph traversal. `top.c` moves data between system memory and accelerator-local storage, starts `bfs`, and copies the results back.

## Step 2: Write the Compute Kernel

`bfs.c` contains the BFS algorithm itself. In the accelerator model, the kernel reads and writes through fixed memory-mapped regions that represent local accelerator-visible storage. In this workload those storage regions are modeled as SALAM "Vars" (RegisterBank-backed memory regions). The code uses the generated `hw_defines.h` to access the bases of these regions:

```c
#include "hw_defines.h"

void
bfs(node_index_t starting_node)
{
    volatile node_t *nodes = (node_t *)NODES;
    volatile edge_t *edges = (edge_t *)EDGES;
    volatile level_t *level = (level_t *)LEVELS;
    volatile edge_index_t *level_counts = (edge_index_t *)LEVELCOUNTS;
    ...
```

The kernel is organized as three nested loops:

* an outer loop over BFS horizons (`horizon < N_LEVELS`, with `N_LEVELS = 10`),
* a middle loop over nodes (`n < N_NODES`, with `N_NODES = 256` for the default `SCALE = 8`),
* an inner loop over the edges of the current node (bounds taken from the graph, so they are data-dependent).

The main loop scans the graph one BFS level at a time:

```c
level[starting_node] = 0;
level_counts[0] = 1;
for (horizon = 0; horizon < N_LEVELS; horizon++) {
    cnt = 0;
    for (n = 0; n < N_NODES; n++) {
        if (level[n] == horizon) {
            edge_index_t tmp_begin = nodes[n].edge_begin;
            edge_index_t tmp_end = nodes[n].edge_end;
            for (e = tmp_begin; e < tmp_end; e++) {
                node_index_t tmp_dst = edges[e].dst;
                level_t tmp_level = level[tmp_dst];

                if (tmp_level == MAX_LEVEL) { // Unmarked
                    level[tmp_dst] = horizon + 1;
                    ++cnt;
                }
            }
        }
    }
    if ((level_counts[horizon + 1] = cnt) == 0)
        break;
}
```

The complete BFS kernel code is under **bfs/hw/bfs.c**.

## Step 3: Write the Controller Kernel

`top.c` acts as a controller kernel. It programs the DMA engine to move input data into accelerator-local storage, launches the BFS kernel, and then moves results back to system memory.

The Top kernel begins by mapping the BFS and DMA memory-mapped registers through the generated address constants:

```c
volatile uint8_t *BFSFlags = (uint8_t *)(BFS);
volatile uint8_t *BFSConfig = (uint8_t *)(BFS + 1);
volatile uint8_t *DmaFlags = (uint8_t *)(DMA_Flags);
volatile uint64_t *DmaRdAddr = (uint64_t *)(DMA_RdAddr);
volatile uint64_t *DmaWrAddr = (uint64_t *)(DMA_WrAddr);
volatile uint32_t *DmaCopyLen = (uint32_t *)(DMA_CopyLen);
```

It then programs the DMA to copy the input graph structures from system memory into accelerator-local storage, polling for completion after each transfer:

```c
// Transfer Nodes
*DmaRdAddr = nodes_addr;
*DmaWrAddr = NODES;
*DmaCopyLen = NODESSIZE;
*DmaFlags = DEV_INIT;
while ((*DmaFlags & DEV_INTR) != DEV_INTR)
    ;
// Transfer Edges
*DmaRdAddr = edges_addr;
*DmaWrAddr = EDGES;
*DmaCopyLen = EDGESSIZE;
*DmaFlags = DEV_INIT;
while ((*DmaFlags & DEV_INTR) != DEV_INTR)
    ;
```

After the inputs are in place, Top writes the starting node into the BFS configuration space, launches the compute kernel, and waits for it to finish. It then DMAs the `level_counts` results back to system memory:

```c
*(node_index_t *)BFSConfig = starting_node;
*BFSFlags = DEV_INIT;
while ((*BFSFlags & DEV_INTR) != DEV_INTR)
    ;

*DmaRdAddr = LEVELCOUNTS;
*DmaWrAddr = level_counts_addr;
*DmaCopyLen = LVLCNTSIZE;
*DmaFlags = DEV_INIT;
while ((*DmaFlags & DEV_INTR) != DEV_INTR)
    ;
```

The complete controller kernel is under **bfs/hw/top.c**.

## Step 4: Describe the Accelerator in config.yml

The workload's **config.yml** describes its devices and instruction timing.

The YAML file has two main roles:

1. Describe the accelerator cluster (what devices exist and how they are wired).
2. Describe per-kernel instruction timing through `runtime_cycles`.

### Describe the accelerator cluster

The `acc_cluster` section declares what hardware blocks exist for the workload. For BFS, that includes:

* a NonCoherent DMA (`Type: NonCoherent`) used for memory transfers,
* two accelerators (`Top` and `bfs`) each bound to an LLVM IR file (`IrPath: hw/top.ll` and `IrPath: hw/bfs.ll`),
* a set of Vars (`Var:` blocks) for accelerator-local storage (NODES/EDGES/LEVELS/LEVELCOUNTS).

This is also where you specify the PIO master bus for each device, PIO window sizes, and (optionally) interrupt numbers.

The following shortened example declares the DMA, the Top controller, the BFS compute kernel, and one accelerator-local RegisterBank:

```yaml
acc_cluster:
    - Name: bfs_clstr
    - DMA:
          - Name: dma
            MaxReqSize: 128
            BufferSize: 256
            PIOMaster: LocalBus
            Type: NonCoherent
            InterruptNum: 95
    - Accelerator:
          - Name: Top
            IrPath: hw/top.ll
            Profile: top
            PIOSize: 37
            PIOMaster: LocalBus
            LocalSlaves: LocalBus
    - Accelerator:
          - Name: bfs
            IrPath: hw/bfs.ll
            Profile: bfs
            PIOSize: 5
            PIOMaster: LocalBus
          - Var:
                - Name: NODES
                  Type: RegisterBank
                  Size: 2048
```

The full file also declares the `EDGES`, `LEVELS`, and `LEVELCOUNTS` RegisterBanks.

### Describe instruction timing

The `hw_config` section provides per-kernel instruction timing. Each
accelerator's `Profile` field selects a key, such as `bfs`, independently
of its LLVM IR file path. The selected mapping supplies per-instruction
`runtime_cycles` overrides.
Instructions without an override retain their `CycleCounts` defaults.
`load` and `store` must stay 0 because memory responses determine their
completion. The supported `br`, `ret`, and `switch_inst` terminators must
also stay 0; nonzero overrides are rejected.

A workload instruction override accepts only `runtime_cycles`. Opcode/FU
mappings belong in the build profile's
[instruction entries](README_SALAM.md#instruction-entries), and FU `limit`
sets allocation. For example:

```yaml
hw_config:
    top:
    bfs:
        instructions:
            add:
                runtime_cycles: 1
            icmp:
                runtime_cycles: 1
```

By default, `hw_config` comes from the YAML document that declares the
cluster. `HWPath` selects another YAML file containing `hw_config`, relative
to the file declaring it. For example, `HWPath: timing/cycles.yml` in
`my-workload/config.yml` selects `my-workload/timing/cycles.yml`.
An accelerator `HWPath` applies to that accelerator; one declared beside
`SysPath` applies to the included clusters. This selects workload timing,
separately from the build-time hardware profile.

### Workload fields

Memory `Size`, stream `StreamSize`, and explicit `BufferSize` values are
positive integer bytes. `StreamSize` is required; a stream buffer defaults
to 256 bytes when `BufferSize` is omitted. Cache `Size` also accepts binary
prefixes such as `32Ki` or `32k`, without a `B` suffix.

Names start with a letter and contain letters, digits, or underscores.
DMA, accelerator, and non-cache variable names must be distinct ignoring
case. Stream-buffer `InCon` and `OutCon` name accelerators in the same
cluster. Accelerator `StreamIn` and `StreamOut` name streaming DMAs; each
direction has one owner. A buffer endpoint cannot also use a DMA stream
connection. `PIOMaster` and `LocalSlaves` support `LocalBus`.

`IrPath` and included `SysPath` paths are relative to the top workload
directory. `HWPath` and memory `Characterization` paths are relative to
their declaring YAML file. Absolute paths are accepted. A single-function
module uses that function as its entry; a module with multiple functions
uses `top`. Set the clock with `Clocks.Compute` below.

### Set the cluster clocks

Each cluster has two clock domains. Its accelerators' `CommInterface` and `LLVMInterface` run on the **Compute** clock (100 MHz by default). Its local bus, DMAs, stream DMAs and buffers, scratchpads and register banks run on the **Fabric** clock (1 GHz by default). The coherency bus and cluster cache stay on the system clock. An optional `Clocks` item in the cluster's list sets either clock as a frequency or a period:

```yaml
acc_cluster:
    - Name: bfs_clstr
    - Clocks:
          Compute: 50MHz
          Fabric: 2ns
```

Latencies count in the owning object's clock domain. Each domain has a
fixed frequency; PIO latencies remain time values.

| Object | Parameter | Clock | Default |
|---|---|---|---|
| `LLVMInterface` | instruction `runtime_cycles` | Compute | per instruction |
| `CommInterface` | `process_cycles` | Compute | 1 |
| `NoncoherentDma` | `process_cycles` | Fabric | 10 |
| `StreamDma` | `process_cycles` | Fabric | 1 |
| `ScratchpadMemory` | `access_cycles`, `access_cycles_var` | Fabric | 2, 0 |
| `RegisterBank` | `access_cycles`, `write_cycles` | Fabric | 10, 10 |
| `StreamBuffer` | `stream_cycles` | Fabric | 1 |

See the [timing model](SALAM_Object_Overview.md#timing-model) for operation
completion and local-memory visibility rules.

### Generated address header

SALAM regenerates a header file that defines the MMIO address map for the cluster. In BFS, this is:

**bfs/bfs_clstr_hw_defines.h**

This file provides constants for:

* DMA MMIO registers (e.g., `DMA_Flags`, `DMA_RdAddr`, …),
* accelerator MMIO bases (e.g., `TOP`, `BFS`),
* local storage bases for Vars (e.g., `NODES`, `EDGES`, …).

Both the host code and accelerator kernels include these definitions, directly or through `hw_defines.h`, so that all components use the same address map. This file is generated by the SALAM configurator; it should not be maintained by hand.

## Step 5: Construct the gem5 System

SALAM runs a bare-metal ARM binary in gem5 full-system mode. Both launchers
read `config.yml` and construct the accelerator devices and connections;
you do not need a separate gem5 configuration for each workload. This guide
uses `run_system.sh`; the
[standard library example](README_SALAM.md#standard-library-bare-metal-example)
provides another launcher for built workloads.

Generate the MMIO header before compiling the host and accelerator sources
so their addresses match the simulated system.

From the gem5 repository root, the BFS hardware header can be generated with:

```bash
util/SALAM-tools/SALAM-Configurator/systembuilder.py \
    --sys-name bfs \
    --bench-path bfs \
    --config-name config.yml
```

This writes `$ACC_BENCH_PATH/bfs/bfs_clstr_hw_defines.h` (for example
`configs/example/gem5_library/salam-benchmarks/src/bfs/bfs_clstr_hw_defines.h`
when `ACC_BENCH_PATH` points at `.../salam-benchmarks/src`). The normal
workflow does not require running it separately because `run_system.sh`,
used in Step 8, invokes the configurator before building and running the
workload.

## Step 6: Write the Host Program

For BFS, the host program is a bare-metal application compiled to an ELF. Under `configs/example/gem5_library/salam-benchmarks/src/bfs`:

**sw/main.elf**

The host program is responsible for:

* allocating or pointing to input/output buffers in CPU-visible memory,
* writing those buffer addresses into the Top accelerator’s MMIO registers,
* launching the Top accelerator by writing its flags register,
* waiting for completion (polling a shared variable or handling an interrupt),
* validating results and exiting.

In BFS, the MMIO layout is accessed using the generated base (`TOP`) plus fixed offsets:

```cpp
volatile uint8_t *top = (uint8_t *)(TOP);
volatile uint32_t *NODES_ADDR = (uint32_t *)(TOP + 1);
volatile uint32_t *EDGES_ADDR = (uint32_t *)(TOP + 9);
volatile uint32_t *LEVEL_ADDR = (uint32_t *)(TOP + 17);
volatile uint32_t *COUNT_ADDR = (uint32_t *)(TOP + 25);
volatile node_index_t *START_ADDR = (node_index_t *)(TOP + 33);
```

After generating the input graph, the host writes the buffer addresses and starting node to these registers. It then writes the run bit to the Top flags register and waits for completion:

```cpp
*NODES_ADDR = (uint32_t)(void *)nodes;
*EDGES_ADDR = (uint32_t)(void *)edges;
*LEVEL_ADDR = (uint32_t)(void *)level;
*COUNT_ADDR = (uint32_t)(void *)level_counts;
*START_ADDR = starting_node;

*top = 0x01;
while (stage < 1) {
    count++;
}

m5_dump_stats(0, 0);
m5_exit(0);
```

The BFS host code is under **bfs/sw/** and links against the shared
bare-metal support code in **common/**, including `syscalls.c`. It includes
gem5's `<gem5/m5ops.h>` from `$M5_PATH/include`.

## Step 7: Compile the Workload

A SALAM workload typically builds two things:

* Accelerator kernels: compiled from `hw/*.c` into LLVM IR (`hw/*.ll`).
* Host program: compiled from `sw/*` into an ELF (`sw/main.elf`).

In the BFS example:

* `bfs/hw/Makefile` produces `bfs.ll` and `top.ll` with `clang -O1 ... -emit-llvm`.
* `bfs/sw/Makefile` produces `main.elf`, and links common syscall stubs from `common/`.

The top-level `bfs/Makefile` builds both directories:

```make
FOLDERS=hw sw

build:
	@( for f in $(FOLDERS); do $(MAKE) CFLAGS="$(CFLAGS)" -C $$f; done )

all: clean build
```

To build the workload directly:

```bash
cd $ACC_BENCH_PATH/bfs
make all
```

This produces `hw/bfs.ll`, `hw/top.ll`, and `sw/main.elf`. This manual build is optional because `run_system.sh` builds the workload by default.

## Step 8: Build gem5 and Run BFS

The recommended entry point is:

**util/SALAM-tools/run_system.sh**

The run script performs the complete flow:

1. Run the configurator to refresh `*_hw_defines.h`.
2. Build the workload (`make all` in the workload directory). `BUILD=True` is the default.
3. Launch gem5 using `configs/SALAM/fs.py` and the host ELF as `--kernel`.

The script uses:

* `M5_PATH` for the gem5 repository,
* `ACC_BENCH_PATH` for the directory containing the workload and the shared `common/` directory.

Point `ACC_BENCH_PATH` at the buildable **`src/`** tree (for example `$M5_PATH/configs/example/gem5_library/salam-benchmarks/src`). The sibling **`workloads/`** tree is a precompiled snapshot without Makefiles and is not the default path when `BUILD=True`.

`--bench` labels the run. `--bench-path` selects
`$ACC_BENCH_PATH/<bench-path>/` and defaults to `--bench` if omitted.
For the in-tree BFS example, use
`--bench bfs --bench-path bfs`.

Example for the in-tree BFS workload:

```bash
cd $M5_PATH
scons build/ARM/gem5.opt --with-salam -j$(nproc)

$M5_PATH/util/SALAM-tools/run_system.sh --bench bfs --bench-path bfs --print
```

The `--print` flag writes gem5 standard output to `debug-trace.txt` in the
output directory. Errors remain on the terminal.

## Step 9: Inspect the Output

When a SALAM workload is run, the run script places output under:

**BM_ARM_OUT/<bench-path>/**

The most common outputs to look at are:

* `stats.txt`: gem5 and SALAM reset-window statistics; see
  [Statistics](#statistics) below.
* `config.ini` and `config.json`: the simulated system configuration.
* `debug-trace.txt` (optional): standard output with `run_system.sh --print`.
* `system.terminal` (if produced by the platform/scripts): the simulated UART/terminal output.

Each accelerator prints a summary when its kernel finishes. BFS produces
separate summaries for the compute kernel and the Top controller. The Top
runtime includes DMA transfers and time waiting for BFS to finish.

The simulated terminal output can also be used to confirm that the host generated its input, launched the accelerator, and completed the job.

## Statistics

`stats.txt` records kernel execution, functional-unit usage, and memory
transfers. It also reports power, energy, and area estimates when
characterization data is supplied; the default profile supplies none.
Check the coverage flags alongside estimates, since missing data leaves
totals incomplete. Kernel summaries cover a full invocation, while
`stats.txt` counters cover the current statistics reset window. Refer to
`stats.txt` for detailed counters and descriptions, and to `config.ini` or
`config.json` for the configured devices and model inputs.

## Optional: Customize the Hardware Profile

`--salam-hw-config` selects a complete build-time hardware profile, separately
from the workload's instruction timing. See
[Hardware Profiles](README_SALAM.md#hardware-profiles) for profile fields and
an example of adding a functional unit.

```bash
scons build/ARM/gem5.opt --with-salam \
    --salam-hw-config=/path/to/salam-hw-config.yml -j$(nproc)
```

To add another workload, provide kernels under `hw/`, a host program under
`sw/`, and a `config.yml` describing the devices and timing. Multiple kernels
can share local memories and DMA within one cluster.
