# gem5-SALAM

gem5-SALAM (System Architecture for LLVM-based Accelerator Modeling), is a novel system architecture designed to enable LLVM-based modeling and simulation of custom hardware accelerators.

# Requirements

- gem5 build dependencies
- LLVM development headers and libraries
- PyYAML, for every `--with-salam` build
- Clang and `gcc-arm-none-eabi`, only when generating LLVM IR or ARM bare-metal ELFs

# gem5-SALAM Setup

## Dependencies

Install the host dependencies for gem5 using the
[gem5 build documentation](https://www.gem5.org/documentation/general_docs/building).

SALAM additionally requires:

- LLVM development headers and libraries (`llvm-dev`)
- PyYAML (`python3-yaml`) for every `--with-salam` build

When building with `--with-salam`, `SConstruct` first uses `LLVM_CONFIG` if it points to an existing `llvm-config` executable, then `llvm-config` on `PATH`, and otherwise searches for the highest available `llvm-config-N` where `N` is 11 through 20.

The current port has been verified to build with LLVM/Clang 14 on Ubuntu 22.04 and LLVM/Clang 18 on Ubuntu 24.04.

To select a particular LLVM installation, set `LLVM_CONFIG` to its
`llvm-config` executable. For compatibility, it is recommended to generate accelerator LLVM IR with a Clang from the same LLVM major version.

On Ubuntu, after the normal gem5 prerequisites:

```bash
sudo apt install llvm-dev python3-yaml
```

Every `--with-salam` build needs PyYAML. Ubuntu provides it as
`python3-yaml`.

`gcc-arm-none-eabi` builds target ARM bare-metal ELF files. Install it,
with clang, only when generating new LLVM IR or those ELFs:

```bash
sudo apt install clang gcc-arm-none-eabi
```

The offline **cacti-SALAM** helper additionally needs `g++-multilib` on
x86_64 Ubuntu; see **Power Modeling using cacti-SALAM**.

# Building gem5-SALAM

When building gem5-SALAM, there are multiple different binary types that can be created. Just like in gem5 the options are debug, opt, fast, prof, and perf. We recommend that users either use the opt or debug builds, as these are the build types we develop and test on.

`--with-salam` currently requires an **ARM** build target.

Below are the bash commands you would use to build the opt or debug binary.

```bash
scons build/ARM/gem5.opt --with-salam -j`nproc`
```

```bash
scons build/ARM/gem5.debug --with-salam -j`nproc`
```

For more information regarding the binary types, and other build information refer to the gem5 build documentation [here](https://www.gem5.org/documentation/general_docs/building).

# Using gem5-SALAM

To use gem5-SALAM you need to define the computation model of your accelerator in C/C++ (or another language that can produce compatible LLVM IR), and compile it to LLVM IR. Any control and dataflow graph optimization (eg. loop unrolling) should be handled by the compiler. You can construct accelerators by associating their LLVM IR with an LLVMInterface and connecting it to the desired CommInterface in the gem5 memory map.

Below are some resources in the gem5-SALAM directory that can be used when getting started:

- **configs/SALAM/HWAcc.py** constructs accelerator SimObjects from the workload YAML at gem5 configuration time.
- The in-tree accelerator example is **BFS** under **configs/example/gem5_library/salam-benchmarks/** (`src/bfs`). Additional SALAM benchmarks will be made available later through [gem5 Resources](https://resources.gem5.org).
- You can also add your own benchmarks. A detailed walkthrough is in **util/SALAM-docs/Building_and_Integrating_Accelerators.md**.

## Quickstart: build and run BFS

The BFS example under **configs/example/gem5_library/salam-benchmarks** shows how to interface with the gem5-SALAM simulation objects.

`run_system.sh` requires **M5_PATH** and **ACC_BENCH_PATH**. Point `ACC_BENCH_PATH` at the buildable `src/` tree (it has Makefiles). The sibling `workloads/` tree is a precompiled snapshot without Makefiles and is not the default path when `BUILD=True` (the script default).

`--bench-path` is the workload directory under `ACC_BENCH_PATH` (host ELF,
IR, and `config.yml`). If omitted, it defaults to the `--bench` value.
`--bench` is a simple identifier such as `bfs` that `run_system.sh` also
passes as `--sys-name` to the configurator and as `--accbench` to
`configs/SALAM/fs.py`. For the in-tree BFS flow, set both to `bfs`.

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src

cd $M5_PATH
scons build/ARM/gem5.opt --with-salam -j`nproc`

# Optional: build the workload yourself first
# cd $ACC_BENCH_PATH/bfs && make

$M5_PATH/util/SALAM-tools/run_system.sh --bench bfs --bench-path bfs
```

With `BUILD=True` (the default), `run_system.sh` runs `make all` in `$ACC_BENCH_PATH/bfs` before launching gem5. To run the precompiled snapshot under `salam-benchmarks/workloads/` instead, point `ACC_BENCH_PATH` there and pass `--no-build`. To inspect the gem5 command line it constructs, see the **RUN_SCRIPT** variable in the shell file.

## Hardware Profiles

`--with-salam` uses the default hardware profile
`src/salam/hw_profiles/default/salam-hw-config.yml`. It generates the
functional-unit and instruction sources under the variant build
directory. Paths in that file are relative to the file. The build
does not modify the profile YAML.

`--salam-hw-config=CONFIG` selects a complete replacement profile.
It does not merge with the default hardware profile, scan
`src/salam/hw_profiles/examples/`, or infer a technology node or
cycle time. Paths in `CONFIG` are relative to that file. The build
does not modify the profile YAML. The selected file must name
`functional_units`, `functional_unit_order`, and `instructions`.

```bash
scons build/ARM/gem5.opt --with-salam \
    --salam-hw-config=/path/to/salam-hw-config.yml -j`nproc`
```

`functional_unit_order` is the generation order. It must list every
non-hidden functional-unit directory once. Hidden names and files that
are not directories are ignored.

The default hardware profile is a 40 nm, 5 ns description.
The workload `Profile` field selects the `hw_config` mapping that
supplies per-kernel `runtime_cycles` through `CycleCounts`. That
workload field is not the build-time hardware profile.

### Required functional units

Handwritten `HWInterface` and `configs/SALAM/HWAccConfig.py` require
these aliases:

`integer_adder`, `integer_multiplier`, `bit_shifter`,
`bitwise_operations`, `float_adder`, `double_adder`,
`float_multiplier`, `float_divider`, `double_multiplier`,
`double_divider`, `bit_register`.

Parameters may change, and a profile may add units. Those eleven
aliases may not be omitted or renamed without editing handwritten
`HWInterface`. Linking a new class does not create an instance.
`HWAccConfig.py` must construct it, for example
`acc.hw_interface.functional_units.float_adder = FloatAdder()`.

### Adding a functional unit

Copy a complete profile, add the unit directory and its YAML, append
the alias to `functional_unit_order`, and edit instruction ownership so
each instruction has one owner. Then build with `--salam-hw-config`
pointing at that copy. `src/salam/hw_profiles/examples/float_trig_sine/`
is an example unit. It is not part of the default hardware profile
and is not scanned.

Remapping a supported opcode onto a functional unit is allowed. A new
YAML instruction name does not add `compute()` support. Instruction
execution stays in handwritten `src/salam/LLVMRead/instruction.cc`.

`functional_unit_limit` is not an effective occupancy limit. The
scheduler does not stall on functional-unit availability.

### Exporting generated sources

`util/SALAM-tools/hw_generator/HWProfileGenerator.py` can write the
same generated sources outside the repository. It takes `--hw-config`
and `--output-dir`, and it refuses to write under `src/`. A normal
`--with-salam` build does not need this tool.

## Power Modeling using cacti-SALAM

**cacti-SALAM** (`util/SALAM-tools/cacti-SALAM`) is an **offline helper** for running CACTI on accelerator YAML configs. It is **not wired into** gem5-SALAM simulation.

On x86_64 Ubuntu, CACTI's 32-bit build needs a 32-bit C/C++ toolchain.
The ordinary gem5 and SALAM packages above do not provide `gcc -m32` /
`g++ -m32`. Install:

```bash
sudo apt install g++-multilib
```

On Ubuntu 22.04 and 24.04, that package also pulls `gcc-multilib`. You do
not need to name `gcc-multilib` as a separate `apt install` argument.
`g++-multilib` is the package that must be installed explicitly.

`setup_cacti_SALAM.py` and `run_cacti_SALAM.py` both require **M5_PATH**
and **ACC_BENCH_PATH** (the same exports used by `run_system.sh`).

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src
cd $M5_PATH/util/SALAM-tools/cacti-SALAM
./setup_cacti_SALAM.py
```

Create `$ACC_BENCH_PATH/benchmarks.list` with three whitespace-separated
fields per line (no spaces within a field):

```
path/to/config.yml <benchmark-name> <config-name>
```

The first field is the path to a workload YAML configuration. It is used as
written when that path exists; otherwise it is resolved relative to
`ACC_BENCH_PATH`. The remaining two fields are labels written to the
`Benchmark` and `Config` columns of `SALAM-out.csv`; they are not YAML keys.
`--bench-list` defaults to `$ACC_BENCH_PATH/benchmarks.list` when omitted.

Then:

```bash
python3 ./run_cacti_SALAM.py --bench-list $ACC_BENCH_PATH/benchmarks.list --delay 1.0
```

`run_cacti_SALAM.py` only models YAML Vars with `Type: SPM`. It checks for
the CACTI binary at `$M5_PATH/ext/mcpat/cacti/cacti` before walking the
YAML, so setup is required even when a config has no SPMs. The in-tree BFS
`config.yml` uses RegisterBanks (`NODES`, `EDGES`, `LEVELS`,
`LEVELCOUNTS`); the helper ignores those, prints that no SPMs were found,
exits 0, and does not write `results/SALAM-out.csv`.

When SPMs are present, results are written to
`util/SALAM-tools/cacti-SALAM/results/SALAM-out.csv`. To use those (or other)
coefficients in a simulation power estimate, add code in
`LLVMInterface::printResults()` in **src/salam/llvm_interface.cc**.

# Resources

## gem5 Documentation

[https://www.gem5.org/documentation/](https://www.gem5.org/documentation/)

## gem5 Tutorial

The gem5 documentation has a [tutorial for working with gem5](http://learning.gem5.org/book/index.html#) that will help get you started with the basics of creating your own sim objects.

## gem5 Bootcamp

The [gem5 Bootcamp](https://bootcamp.gem5.org/) provides hands-on tutorials
for learning and using gem5.

## Building and Integrating Accelerators in gem5-SALAM

We have written a guide that walks through the in-tree BFS example. This will help you get started with creating your own benchmarks and systems. It can be viewed at **util/SALAM-docs/Building_and_Integrating_Accelerators.md**.

## SALAM Object Overview

The overview at **util/SALAM-docs/SALAM_Object_Overview.md** covers what various Sim Objects in gem5-SALAM are and their purpose.

## Full-system OS Simulation

Full-system resources such as kernels and disk images are available through
[gem5 Resources](https://resources.gem5.org/).
Devices operate in the physical memory address space.
