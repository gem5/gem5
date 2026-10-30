# gem5-SALAM

gem5-SALAM (System Architecture for LLVM-based Accelerator Modeling) models
custom hardware accelerators from LLVM IR.

# Dependencies

Install the normal [gem5 build dependencies](https://www.gem5.org/documentation/general_docs/building),
then add LLVM development headers/libraries and PyYAML:

```bash
sudo apt install llvm-dev python3-yaml
```

To build the in-tree bare-metal workloads, also install Clang, the ARM
cross-compiler, and its C/C++ libraries:

```bash
sudo apt install clang gcc-arm-none-eabi libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib
```

Set `LLVM_CONFIG` to select a particular `llvm-config` executable.
Otherwise, SCons uses `llvm-config` on `PATH`, then searches for the highest
available `llvm-config-N` with N from 11 through 20. Generate accelerator
IR with a Clang from the same LLVM major version.

The offline CACTI helper additionally needs `g++-multilib` on x86_64
Ubuntu; see [Offline CACTI characterization](#offline-cacti-characterization).

# Building gem5-SALAM

gem5 provides `debug`, `opt`, and `fast` binaries. Use `opt` for ordinary
runs or `debug` when debugging.

`--with-salam` currently requires an **ARM** build target.

Below are the bash commands you would use to build the opt or debug binary.

```bash
scons build/ARM/gem5.opt --with-salam -j$(nproc)
```

```bash
scons build/ARM/gem5.debug --with-salam -j$(nproc)
```

For more information regarding the binary types, and other build information refer to the gem5 build documentation [here](https://www.gem5.org/documentation/general_docs/building).

# Using gem5-SALAM

Write the accelerator kernel in C/C++ or another language that emits
compatible LLVM IR. The compiler handles graph transformations such as
loop unrolling; SALAM executes the resulting control and dataflow graph.

Use the in-tree BFS workload to get started. The
[accelerator guide](Building_and_Integrating_Accelerators.md) walks through
its kernels, host program, and YAML configuration. See the
[object overview](SALAM_Object_Overview.md) for device behavior and supported
execution semantics.

## Quickstart: build and run BFS

The BFS example under **configs/example/gem5_library/salam-benchmarks** shows how to interface with the gem5-SALAM simulation objects.

`run_system.sh` requires **M5_PATH** and **ACC_BENCH_PATH**. Point `ACC_BENCH_PATH` at the buildable `src/` tree (it has Makefiles). The sibling `workloads/` tree is a precompiled snapshot without Makefiles and is not the default path when `BUILD=True` (the script default).

`--bench-path` is the workload directory under `ACC_BENCH_PATH` (host ELF,
IR, and `config.yml`). If omitted, it defaults to the `--bench` value.
For the in-tree BFS flow, use `--bench bfs --bench-path bfs`.

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src

cd $M5_PATH
scons build/ARM/gem5.opt --with-salam -j$(nproc)

$M5_PATH/util/SALAM-tools/run_system.sh --bench bfs --bench-path bfs
```

The script generates MMIO headers and builds the workload before launching
gem5. To run the precompiled snapshot under `salam-benchmarks/workloads/`,
point `ACC_BENCH_PATH` there and pass `--no-build`.

## Standard library bare-metal example

Build gem5 with `--with-salam`, then provide a built workload directory
containing its ELF, IR, and YAML:

```bash
./build/ARM/gem5.opt -d m5out-stdlib \
    configs/example/gem5_library/salam/arm-baremetal-salam.py \
    --binary /path/to/bfs/sw/main.elf --accpath /path/to/bfs
```

`--acccfg` selects a YAML filename relative to `--accpath`; its default is
`config.yml`. `--max-ticks` optionally bounds the simulation. The workload
files can be outside the source checkout. Copy the example script too when
running away from the checkout.

To construct another configuration, import `SALAMArmBoard` from
`gem5.salam.board`, supply a processor, memory, and classic cache hierarchy
with a `membus`, call `set_bare_metal_binary(path)`, then
`attach_accelerators(workload_directory)`, before starting the simulator.
Ruby hierarchies are rejected. The board enters the ARM ELF directly,
without a Linux kernel, bootloader, or device tree. UART 0 terminates on
end-of-transmission; the supplied workloads also use `m5_exit`.

The example uses a 2 GHz O3 CPU, a 1 GHz board, private L1 caches, a shared
L2, and 16 GiB of DDR4 memory.

## Statistics and reset windows

SALAM writes reset-window counters to `stats.txt`. Resetting statistics
preserves device state and queued work, so requests and responses can fall
in different windows. The printed kernel summary covers the full invocation.

| Object | Useful counters |
|---|---|
| `LLVMInterface` | invocations, compute passes, issued/completed operations, FU denials |
| `HWInterface.<unit>` | instance count, admissions, denials, utilization |
| `CommInterface` | logical requests, send attempts, accepted packets, responses |
| DMA and stream devices | transfers, packets, bytes, active time |
| `SALAMPowerModel` | known area/energy/power and independent coverage flags |

See [statistics semantics](SALAM_Object_Overview.md#statistics) for counter
definitions and the [workload guide](Building_and_Integrating_Accelerators.md#statistics)
for inspecting run outputs.

## DMA configuration

A DMA's `Type` must be exactly `NonCoherent` or `Stream`. `Name` and
`PIOMaster` are required. `BufferSize` and `MaxReqSize`, when supplied, must
be positive integers in bytes; omitted values retain the device defaults.
The default buffer capacity is 1024 bytes, and the default request limit is
the parent system's cache-line size. Interrupt numbers are optional
nonnegative integers: `InterruptNum` for a NonCoherent DMA, and `ReadInt`
and `WriteInt` for a Stream DMA. Fields for the other DMA type are rejected.

For a Stream DMA, `BufferSize` sets **both** its read and write buffer
capacities, and `MaxReqSize` sets its DMA request limit. The stream width
is 8 bytes, and the maximum number of pending DMA requests is 8.

NonCoherent DMA moves a complete request chunk between its two FIFOs.
In every memory mode, `min(length, MaxReqSize)` must fit both FIFOs, each
of which has half of `BufferSize`. The launch rejects an impossible chunk
before either FIFO starts. Short transfers remain valid when their actual
chunk fits even if `MaxReqSize` is larger than the capacity.

Stream DMA applies the capacity check on its timing path; its
`atomic_noncaching` path can use smaller stream beats.

See the [test instructions](../../tests/gem5/salam/README.md) to run SALAM
regression suites.

## Hardware Profiles

`--with-salam` uses the default hardware profile
`configs/SALAM/hw_profiles/default/salam-hw-config.yml`. It embeds ordered
functional-unit and instruction parameter records as `m5._salam_profile`,
generated under the variant build directory. Paths in the profile are
relative to that file. The build does not modify the profile YAML, and the
simulator does not read that YAML at runtime.

`--salam-hw-config=CONFIG` selects a complete replacement profile with
`functional_units`, `functional_unit_order`, and `instructions`. Paths are
relative to that file. Profiles are not merged, and example units are
included only when added explicitly.

```bash
scons build/ARM/gem5.opt --with-salam \
    --salam-hw-config=/path/to/salam-hw-config.yml -j$(nproc)
```

Repeat both `--with-salam` and `--salam-hw-config=CONFIG` on every build,
including incremental builds. `--with-salam` alone selects the default
profile. Omitting `--with-salam` disables SALAM; passing a custom profile
without it is an error.

`functional_unit_order` is the collection order. It must list every
non-hidden functional-unit directory once. Hidden names and files that
are not directories are ignored.

The workload `Profile` field selects the `hw_config` mapping that
supplies per-kernel `runtime_cycles` through `CycleCounts`. That
workload field is not the build-time hardware profile.

### Functional unit construction

Each accelerator receives independent functional units and instruction
mappings from the selected profile. Profiles can add, remove, or rename
units. Every instruction's `functional_unit` must name a unit's `enum_value`
or be 0 for none.

Programmatic configurations use the common `FunctionalUnit` and
`InstructionConfig` types. Entry aliases must be public Python identifiers
that do not conflict with collection attributes.

### Adding a functional unit

Copy a complete profile, add the unit directory and its YAML, append
the alias to `functional_unit_order`, and edit instruction ownership so
each instruction has one owner. Then build with `--salam-hw-config`
pointing at that copy. `configs/SALAM/hw_profiles/examples/float_trig_sine/`
is an example unit. It is not part of the default hardware profile
and is not scanned.

Remapping a supported opcode changes its functional unit, not its operation:
an `fsub` still subtracts. A new operation needs dispatch in
`LLVMInterface::createInstruction()` in `src/salam/llvm_interface.cc` and
an implementation in `src/salam/LLVMRead/instruction.cc`.

Nonrecursive direct calls to functions defined in the same IR module are
supported when their bodies use supported instructions. Indirect calls and
calls to declarations are rejected; this includes external libm calls and
LLVM intrinsics emitted as declarations. Naming a unit after such a
function does not add support for calling it.

For example, to run `fsub` on the example unit, copy
`configs/SALAM/hw_profiles/default/` and, in the copy:

1. copy `configs/SALAM/hw_profiles/examples/float_trig_sine/` into
   `functional_units/`;
2. remove `fsub` from the `instructions` list in
   `functional_units/double_adder/double_adder.yml`;
3. replace `- none` with `- fsub` in
   `functional_units/float_trig_sine/float_trig_sine.yml`;
4. append `float_trig_sine` to `functional_unit_order` in
   `salam-hw-config.yml`.

Build with `--salam-hw-config` set to the copy's
`salam-hw-config.yml` and include `--with-salam`. If step 3 is omitted, the
build still succeeds: `fsub` keeps the `double_adder` mapping (enum 6)
configured in `inst_list.yml`.

Set the subtraction latency in the workload's selected profile, for example:

```yaml
hw_config:
    my_kernel:
        instructions:
            fsub:
                runtime_cycles: 3
```

Here `my_kernel` must match the accelerator's `Profile` field. The unit's
`cycles` and `stages` are metadata; they do not set execution latency.

BFS has no floating-point operations. Use a workload whose IR contains
`fsub` to observe the remapping. The kernel at
`tests/gem5/salam/harness/kernels/fsub.c` shows a minimal subtraction.
For a workload under `$ACC_BENCH_PATH/fsub`, enable inventory tracing with:

```bash
$M5_PATH/util/SALAM-tools/run_system.sh --bench fsub --bench-path fsub \
    -f SALAMFUInventory --print
```

With the example's `limit: 0`, exactly one static `fsub` in the reachable
kernel graph gives `float_trig_sine (17): 1 instances, 1 static instructions`
in the trace. More static uses allocate more instances. Check
`<accelerator>.hw_interface.float_trig_sine.admissions` in `stats.txt` to
confirm that operations used the unit; inventory alone shows allocation.

### Functional unit scheduling

Each accelerator has its own functional units. A ready compute
operation that names a unit issues only when an instance of that unit
can admit it; otherwise it stays ready and waits. Loads, stores,
branches, calls and phis never use a unit, and the generator's
`functional_unit` for them must be 0.

These unit parameters schedule it:

- `limit` is the unit's instance count. `limit: 0` gives one instance
  per static instruction that uses the unit in the top function and
  the functions it reaches through direct calls; a function called
  from several places counts once. A unit that no such instruction
  uses has no instances, and any operation that needs it is rejected.
- `initiation_interval`, if given, makes the unit pipelined: an
  instance admits an operation once that many cycles have passed
  since its previous admission, however many operations it has in
  flight. Without it an instance admits one operation at a time, after
  the previous operation completes. Each physical instance admits at
  most once per compute edge. A zero-cycle operation completes and
  releases its token immediately; its instance can next admit on the
  following compute edge. Operations with unit identifier 0 have no
  physical admission restriction.

Execution latency comes from `CycleCounts`, with per-kernel overrides from
`hw_config.<Profile>.instructions.<instruction>.runtime_cycles` in the
workload. Overrides accept only `runtime_cycles`. FU `limit` sets inventory;
`initiation_interval` sets admission. FU `cycles`, `stages`, and datatype
fields are descriptive metadata. See
[execution semantics](SALAM_Object_Overview.md#execution-semantics)
for control flow and memory ordering.

With `lockstep_mode` (the default) the accelerator issues nothing new
until its compute and memory queues are empty, so an
`initiation_interval` shorter than the latency only overlaps
operations with `lockstep_mode=False`.

The `SALAMFUInventory` debug flag prints each unit's instance count
when an accelerator starts and the number of times a ready operation
waited for a unit when it finishes. The instance counts must be the
same for every invocation.

For example, three independent multiplies on one `integer_multiplier`
instance with a latency of 4 issue 4 cycles apart. With
`initiation_interval: 1` and `lockstep_mode=False` they issue in
consecutive cycles.

### Instruction entries

The profile's `instructions` file, `inst_list.yml` in the default
profile, has one entry per instruction:

```yaml
instructions:
    add:
        functional_unit: 1
        opcode_num: 13
```

- The key names an instruction mapping, such as `add`.
- `opcode_num` is LLVM's `Instruction::getOpcode()` value from
  `llvm/IR/Instruction.def` for the LLVM version used to build gem5. Copy
  the correct number for the operation; the generator does not check that
  the entry name and number identify the same LLVM instruction.
- `functional_unit` is a functional unit's `enum_value`; 0 means none.
  The generator sets this value in memory from the `enum_value` of the
  unit whose `instructions` list contains the instruction. It does not
  modify `inst_list.yml`. An instruction no unit lists keeps its
  configured value. The generator rejects an instruction listed by two
  units. To remap an instruction, move its name from one unit's list
  to another's.

The build-profile entry contains only the opcode and FU mapping. Execution
latency and instance limits use the authoritative settings described in
[Functional unit scheduling](#functional-unit-scheduling).

### Exporting profile data

`util/SALAM-tools/hw_generator/HWProfileGenerator.py` can write the
same embedded profile-data module outside the repository. It takes
`--hw-config` and `--output-dir`, and it refuses to write under `src/`.
A normal `--with-salam` build does not need this tool.

## Offline CACTI characterization

The `setup_cacti_SALAM.py` and `run_cacti_SALAM.py` scripts below are
offline batch helpers for workload YAML; gem5 does not run CACTI during
simulation.

On x86_64 Ubuntu, CACTI's 32-bit build needs a 32-bit C/C++ toolchain.
The ordinary gem5 and SALAM packages above do not provide `gcc -m32` /
`g++ -m32`. Install:

```bash
sudo apt install g++-multilib
```

`setup_cacti_SALAM.py` and `run_cacti_SALAM.py` both require **M5_PATH**
and **ACC_BENCH_PATH** (the same exports used by `run_system.sh`).
Build CACTI with the setup helper only for configurations containing SPMs:

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

`run_cacti_SALAM.py` only models YAML Vars with `Type: SPM` and `Name`,
`Size`, and `Ports` fields. The CACTI binary at
`$M5_PATH/ext/mcpat/cacti/cacti` is needed only when a configuration has
such SPMs. The in-tree BFS configuration uses RegisterBanks; the helper
skips it without running CACTI or producing a new `results/SALAM-out.csv`.

When SPMs are present, results are written to
`util/SALAM-tools/cacti-SALAM/results/SALAM-out.csv`.

# Resources

## gem5 Documentation

[https://www.gem5.org/documentation/](https://www.gem5.org/documentation/)

## gem5 Tutorial

The gem5 documentation has a [tutorial for working with gem5](https://www.gem5.org/documentation/learning_gem5/introduction) that will help get you started with the basics of creating your own sim objects.

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
