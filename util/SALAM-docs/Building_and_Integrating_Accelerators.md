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

On the hardware side, SALAM models the datapath through a functional-unit inventory, per-instruction cycle counts, and device characterization at a chosen technology node. The in-tree default targets a 40nm node with a 5ns cycle-time budget. Changing that model is optional and is described later in this guide.

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

Older versions of SALAM used per-kernel INI files to describe cycle counts and device parameters. The current flow uses a single YAML file per workload: **config.yml**.

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
                  Ports: 1
                  ReadyMode: false
```

The full file also declares the `EDGES`, `LEVELS`, and `LEVELCOUNTS` RegisterBanks.

### Describe instruction timing

The `hw_config` section provides per-kernel instruction timing. Each accelerator's `Profile` field names the key to use. For example, `Profile: bfs` selects the `bfs:` section. The IR path is only the location of the bitcode.

Each listed instruction includes opcode and functional-unit metadata along with its execution latency. In the current flow, `AccConfig()` uses the `runtime_cycles` values from this section to configure per-kernel instruction timing. For example:

```yaml
hw_config:
    top:
    bfs:
        instructions:
            add:
                functional_unit: 1
                functional_unit_limit: 0
                opcode_num: 13
                runtime_cycles: 1
            icmp:
                functional_unit: 0
                functional_unit_limit: 0
                opcode_num: 53
                runtime_cycles: 1
```

`Profile` selects the `hw_config` key. It does not have to equal the IR file name. The BFS example uses `bfs` for `hw/bfs.ll`.

### Generated address header

SALAM regenerates a header file that defines the MMIO address map for the cluster. In BFS, this is:

**bfs/bfs_clstr_hw_defines.h**

This file provides constants for:

* DMA MMIO registers (e.g., `DMA_Flags`, `DMA_RdAddr`, …),
* accelerator MMIO bases (e.g., `TOP`, `BFS`),
* local storage bases for Vars (e.g., `NODES`, `EDGES`, …).

Both the host code and accelerator kernels include these definitions, directly or through `hw_defines.h`, so that all components use the same address map. This file is generated by the SALAM configurator; it should not be maintained by hand.

## Step 5: Generate the gem5 System

SALAM uses gem5 full-system mode to run a bare-metal ARM binary on a modeled platform. The key difference from the standard gem5 full-system scripts is that SALAM inserts an accelerator cluster into the system and connects it to the system buses.

Rather than requiring you to hand-write a new gem5 config for each workload, SALAM parses `config.yml` at gem5 configuration time and constructs the accelerator cluster.

The main pieces involved are:

* **util/SALAM-tools/SALAM-Configurator/systembuilder.py**
  Parses `config.yml`, assigns addresses, and generates:

  * `$ACC_BENCH_PATH/<bench-path>/{cluster}_hw_defines.h` (host/accelerator MMIO macros)

* **configs/SALAM/fs.py**
  The checked-in full-system gem5 configuration. On ARM it calls `HWAcc.makeHWAcc()`.

* **configs/SALAM/HWAcc.py**
  Constructs AccCluster, DMA, CommInterface, and memory SimObjects from the same YAML used for headers.

* **configs/SALAM/HWAccConfig.py**
  Contains `AccConfig(...)`, which binds a CommInterface to:

  * an LLVM IR kernel (via `LLVMInterface.in_file`)
  * a timing model loaded from `config.yml` (runtime cycles per instruction)
  * the HWModeling instruction configuration used during execution

In `HWAcc.py`, each accelerator is instantiated as a `CommInterface`, then configured by calling `AccConfig(...)` with its `.ll` path and the YAML config file.

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

The BFS host code is under **bfs/sw/**, and links against the shared bare-metal support code in **common/** (e.g., `syscalls.c`). Host code includes gem5’s `<gem5/m5ops.h>` from `$M5_PATH/include` rather than a copy under `common/`.

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

`--bench-path` selects `$ACC_BENCH_PATH/<bench-path>/`. If omitted, it
defaults to `--bench`. For the in-tree BFS example, use
`--bench bfs --bench-path bfs`.

Example for the in-tree BFS workload:

```bash
cd $M5_PATH
scons build/ARM/gem5.opt --with-salam -j$(nproc)

$M5_PATH/util/SALAM-tools/run_system.sh --bench bfs --bench-path bfs --print
```

The `--print` flag redirects gem5 stdout/stderr to the output directory so the SALAM performance summary is easy to inspect. See **util/SALAM-docs/README_SALAM.md** for the current LLVM/Clang and ARM cross-compiler setup.

## Step 9: Inspect the Output

When a SALAM workload is run, the run script places output under:

**BM_ARM_OUT/<bench-path>/**

The most common outputs to look at are:

* `stats.txt`: gem5 statistics (and SALAM object stats if enabled/dumped).
* `debug-trace.txt` (optional): if `run_system.sh --print` is used, stdout/stderr are redirected here.
* `system.terminal` (if produced by the platform/scripts): the simulated UART/terminal output.

Each `LLVMInterface` prints a summary when its kernel finishes. The fields come from `LLVMInterface::printResults()` in **src/salam/llvm_interface.cc**. BFS produces separate summaries for the compute kernel and the Top controller. The Top runtime includes DMA control and the call to the BFS kernel.

The simulated terminal output can also be used to confirm that the host generated its input, launched the accelerator, and completed the job.

## Optional: Customize the Hardware Profile

Plain `--with-salam` uses the default hardware profile
`src/salam/hw_profiles/default/salam-hw-config.yml`. It generates
sources under the variant build directory. `--salam-hw-config=CONFIG`
selects a complete replacement profile. Paths in `CONFIG` are relative
to that file. The build does not modify the profile YAML. It does not
merge with the default hardware profile, scan `examples/`, or infer a
technology node or cycle time. See **util/SALAM-docs/README_SALAM.md**
for the eleven required aliases and for how to append a unit to
`functional_unit_order`.

A linked SimObject class is not an instance. `configs/SALAM/HWAccConfig.py`
must construct every unit the profile contains. Remapping a supported
opcode is allowed. A new YAML instruction name does not add `compute()`
support.

The workload `Profile` field selects the **config.yml**
`hw_config` mapping. That mapping supplies per-kernel execution latency
through `CycleCounts`. It is not the build-time hardware profile. The
**workloads/** tree is a ready-to-run artifact snapshot. It does not
include the hardware profile.

```bash
scons build/ARM/gem5.opt --with-salam \
    --salam-hw-config=/path/to/salam-hw-config.yml -j$(nproc)
```

The same workload structure used for BFS—kernel source under `hw/`, a host program under `sw/`, and a `config.yml` that describes the cluster and timing—applies to other kernels as well (for example GEMM or stencil). Once that pattern is familiar, adding a new accelerator is largely a matter of writing a new kernel and updating the YAML, rather than rebuilding the system from scratch. Multiple kernels can also be placed in one AccCluster and connected through shared local memories and DMA, which makes it practical to assemble end-to-end workloads such as MobileNet from a set of cooperating accelerators.

Additional SALAM benchmarks beyond the in-tree BFS example will be made available later through gem5 Resources.
