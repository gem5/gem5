# Building and Integrating Accelerators in gem5-SALAM

This guide walks through how SALAM specifies, builds, and runs an accelerator-driven workload in gem5, using BFS as the example.

At a high level, gem5 runs a normal CPU program (an ELF binary). That CPU program configures and launches one or more SALAM accelerators via MMIO. Each SALAM accelerator executes an LLVM IR “kernel” inside gem5 using SALAM’s timing model.

In the BFS example, the workload is split into:

* a host program (`sw/main.elf`) that runs on the simulated CPU,
* one or more accelerator kernels (`hw/*.ll`) that SALAM executes inside gem5,
* a YAML specification (`config.yml`) that describes the accelerator cluster topology and timing model,
* an auto-generated header (`*_hw_defines.h`) that keeps software-visible MMIO addresses consistent with gem5’s address map.

## Writing the Accelerator Code

The BFS example is located under:

**configs/example/gem5_library/salam-benchmarks/src/bfs**

It contains a folder for accelerator kernel code (`hw`) and a folder for the host program (`sw`). SALAM workloads typically make a few key design choices early (how kernels are partitioned, what local storage exists, whether DMA is used, and how the host programs and synchronizes with the accelerator).

In BFS, the accelerator side is written as two kernels:

* `bfs.c` implements the compute kernel.
* `top.c` orchestrates data movement and launches the compute kernel.

### bfs.c

`bfs.c` contains the BFS algorithm itself. In the accelerator model, the kernel reads and writes through fixed memory-mapped regions that represent local accelerator-visible storage. In this workload those storage regions are modeled as SALAM "Vars" (RegisterBank-backed memory regions). The code uses the generated `hw_defines.h` to access the bases of these regions.

The complete BFS kernel code is under **bfs/hw/bfs.c**.

### top.c

`top.c` acts as a controller kernel. It programs the DMA engine to move input data into accelerator-local storage, launches the BFS kernel, and then moves results back to system memory.

In the BFS example, `top.c` programs the DMA through its MMIO registers and polls for completion using flag bits. It also writes a runtime parameter (the starting node) into the BFS accelerator's configuration space before launching it. The complete top kernel code is under **bfs/hw/top.c**.

## Configuration Files (config.yml and generated address headers)

Older versions of SALAM used per-kernel INI files to describe cycle counts and device parameters. The current flow uses a single YAML file per workload: **config.yml**.

The YAML file has two main roles:

1. Describe the accelerator cluster (what devices exist and how they are wired).
2. Describe the timing model (how LLVM IR instructions map to functional units and runtime cycles).

### Accelerator cluster description (acc_cluster)

The `acc_cluster` section declares what hardware blocks exist for the workload. For BFS, that includes:

* a NonCoherent DMA (`Type: NonCoherent`) used for memory transfers,
* two accelerators (`Top` and `bfs`) each bound to an LLVM IR file (`IrPath: hw/top.ll` and `IrPath: hw/bfs.ll`),
* a set of Vars (`Var:` blocks) for accelerator-local storage (NODES/EDGES/LEVELS/LEVELCOUNTS).

This is also where you specify the PIO master bus for each device, PIO window sizes, and (optionally) interrupt numbers.

### Timing model description (hw_config)

The `hw_config` section provides per-kernel instruction timing. The keys under `hw_config` correspond to the LLVM IR file base names. For example, `hw/bfs.ll` maps to the `bfs:` section.

Each listed instruction gives SALAM enough information to model execution cost and (optionally) structural limits. In the BFS example, each instruction entry includes fields like `runtime_cycles`, plus metadata such as `functional_unit` and `opcode_num`.

### Generated header: *_hw_defines.h

SALAM regenerates a header file that defines the MMIO address map for the cluster. In BFS, this is:

**bfs/bfs_clstr_hw_defines.h**

This file provides constants for:

* DMA MMIO registers (e.g., `DMA_Flags`, `DMA_RdAddr`, …),
* accelerator MMIO bases (e.g., `TOP`, `BFS`),
* local storage bases for Vars (e.g., `NODES`, `EDGES`, …).

Both the host code and the accelerator kernels include these definitions (directly or via `hw_defines.h`) so that the same MMIO map is used end-to-end.

## Constructing the System

SALAM uses gem5 full-system mode to run a bare-metal ARM binary on a modeled platform. The key difference from the standard gem5 full-system scripts is that SALAM inserts an accelerator cluster into the system and connects it to the system buses.

Rather than requiring you to hand-write a new gem5 config for each workload, SALAM generates per-workload gem5 Python configs from the YAML file.

The main pieces involved are:

* **util/SALAM-tools/SALAM-Configurator/systembuilder.py**
  Parses `config.yml`, assigns addresses, and generates:

  * `configs/SALAM/<bench>.py` (accelerator cluster construction)
  * `configs/SALAM/fs_<bench>.py` (full-system wrapper)

* **util/SALAM-tools/SALAM-Configurator/fs_template.py**
  The template full-system script that imports `<bench>.py` and calls `makeHWAcc()`.

* **configs/SALAM/HWAccConfig.py**
  Contains `AccConfig(...)`, which binds a CommInterface to:

  * an LLVM IR kernel (via `LLVMInterface.in_file`)
  * a timing model loaded from `config.yml` (runtime cycles per instruction)
  * the HWModeling instruction/functional-unit objects used during execution

In the generated `<bench>.py`, each accelerator is instantiated as a `CommInterface`, then configured by calling `AccConfig(...)` with its `.ll` path and the YAML config file.

## Writing the Host Code

For BFS, the host program is a bare-metal application compiled to an ELF:

**bfs/sw/main.elf**

The host program is responsible for:

* allocating or pointing to input/output buffers in CPU-visible memory,
* writing those buffer addresses into the Top accelerator’s MMIO registers,
* launching the Top accelerator by writing its flags register,
* waiting for completion (polling a shared variable or handling an interrupt),
* validating results and exiting.

In BFS, the MMIO layout is accessed using the generated base (`TOP`) plus fixed offsets. The host writes the input buffer addresses and the starting node to the Top accelerator’s MMIO window, then sets the Top flags to start execution.

The BFS host code is under **bfs/sw/**, and links against the shared bare-metal support code in **common/** (e.g., `m5ops.h`, `syscalls.c`).

## Compiling the Workload

A SALAM workload typically builds two things:

* Accelerator kernels: compiled from `hw/*.c` into LLVM IR (`hw/*.ll`).
* Host program: compiled from `sw/*` into an ELF (`sw/main.elf`).

In the BFS example:

* `bfs/hw/Makefile` produces `bfs.ll` and `top.ll`.
* `bfs/sw/Makefile` produces `main.elf`, and links common syscall stubs from `common/`.

The top-level `bfs/Makefile` typically dispatches to both subdirectories so a single `make all` builds the full workload.

## Running the Workload

The recommended entry point is:

**util/SALAM-tools/run_system.sh**

This script automates the end-to-end flow:

1. Run the configurator to generate `configs/SALAM/<bench>.py`, `configs/SALAM/fs_<bench>.py`, and the `*_hw_defines.h` header.
2. Build the workload (`make all` in the workload directory).
3. Launch gem5 using the generated `fs_<bench>.py` config and the host ELF as `--kernel`.

The script uses:

* `M5_PATH` for the gem5 repository,
* `ACC_BENCH_PATH` for the directory containing the workload and the shared `common/` directory.

It also supplies a placeholder disk image (`common/fake.iso`) because the platform configuration expects a disk image even in bare-metal mode.

## Workload Output

When a SALAM workload is run, the run script places output under:

**BM_ARM_OUT/<bench-path>/**

The most common outputs to look at are:

* `stats.txt`: gem5 statistics (and SALAM object stats if enabled/dumped).
* `debug-trace.txt` (optional): if `run_system.sh --print` is used, stdout/stderr are redirected here.
* `system.terminal` (if produced by the platform/scripts): the simulated UART/terminal output.
