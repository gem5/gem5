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
x86_64 Ubuntu; see
[Offline CACTI characterization](#offline-cacti-characterization).

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

SALAM requires equal pointer and GEP index widths in each supported address
space used by the accelerator graph. Unequal widths are rejected before
execution.

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

## Standard library bare-metal example

The standard library launcher uses the same embedded `gem5.salam`
construction code as the legacy launcher. Build with `--with-salam`, then
provide a built workload directory containing its ELF, IR, and YAML:

```bash
./build/ARM/gem5.opt -d m5out-stdlib \
    configs/example/gem5_library/salam/arm-baremetal-salam.py \
    --binary /path/to/bfs/sw/main.elf --accpath /path/to/bfs
```

`--acccfg` selects a YAML filename relative to `--accpath`; its default is
`config.yml`. `--max-ticks` optionally bounds the simulation. The workload
files can be outside the source checkout. The binary embeds the parsers
and construction modules; it does not import them from `util/` or
`configs/SALAM` at runtime. Copy the example script too when running away
from the checkout.

To construct another configuration, import `SALAMArmBoard` from
`gem5.salam.board`, supply a processor, memory, and classic cache hierarchy
with a `membus`, call `set_bare_metal_binary(path)`, then
`attach_accelerators(workload_directory)`, before starting the simulator.
Ruby hierarchies are rejected. The board enters the ARM ELF directly,
without a Linux kernel, bootloader, or device tree. UART 0 terminates on
end-of-transmission; the supplied workloads also use `m5_exit`.

The example uses ordinary board and cache-hierarchy defaults with a 2 GHz
CPU, a 1 GHz board, and 16 GiB of DDR4 memory. The dedicated
`tests/gem5/salam/legacy_parity.py` configuration selects the legacy
launcher's cache, clock, platform, and bus settings. Parity suites use the
same binary and workload artifacts, compare effective configurations first,
and require exact architectural statistics, UART output, invocation cycles,
and exit ticks. Only hierarchy labels, output paths, and host-performance
measurements are excluded from that comparison.

## Statistics and reset windows

SALAM statistics are written to gem5's `stats.txt`. Counters report the
current reset window. Resetting statistics does not reset the accelerator,
change queued events, release functional units, or empty buffers. A
response after a reset belongs to the new window even if its request was
accepted before the reset. Consequently, requests and responses need not
balance within an individual window.

The accelerator's printed summary covers its current invocation, including
work before a statistics reset. `Compute passes` counts scheduler passes,
including the first and final pass. `Invocation duration` and `Runtime`
measure elapsed simulator time from accepted launch to completion, in ticks
and microseconds respectively. A pass count is not an elapsed-cycle count:
14 passes on consecutive edges span 13 clock periods. The launch debug trace
also reports the interval from the first compute pass separately. These
per-invocation values are distinct from the reset-window counters. PPA
statistics report functional-unit and modeled-storage estimates with
separate area, dynamic-energy, and leakage coverage. Missing characterization
produces an explicit incomplete estimate. Set `report_details=True` on the
desired `SALAMPowerModel` children for invocation and statistics-window
details, including physical inventory, SSA slots, and missing coverage
reasons. Detailed output defaults to off; native statistics and strict
coverage checks remain active. See
[Power, performance, and area](PPA_and_Validation.md) for the coefficient
schema, units, model limits, and explicit reporting commands.

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
  `beats` counts FIFO-to-FIFO moves. The historical `writePackets` name counts
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

## DMA configuration

A DMA's `Type` must be exactly `NonCoherent` or `Stream`. `Name` and
`PIOMaster` are required. `BufferSize` and `MaxReqSize`, when supplied, must
be positive integers in bytes; omitted values retain the device defaults.
The default buffer capacity is 1024 bytes, and the default request limit is
the parent system's cache-line size. Interrupt numbers are optional
nonnegative integers: `InterruptNum` for a NonCoherent DMA, and `ReadInt`
and `WriteInt` for a Stream DMA. Fields for the other DMA type are rejected.

For a Stream DMA, `BufferSize` sets **both** its read and write buffer
capacities, and `MaxReqSize` sets its actual DMA request limit. Earlier
configuration code incorrectly used `BufferSize` as the stream width and
ignored `MaxReqSize`. The stream width remains the device default of 8
bytes; the maximum number of pending DMA requests remains 8. The PIO
register span does not set that request count.

NonCoherent DMA moves a complete request chunk between its two FIFOs.
In every memory mode, `min(length, MaxReqSize)` must fit both FIFOs, each
of which has half of `BufferSize`. The launch rejects an impossible chunk
before either FIFO starts. Short transfers remain valid when their actual
chunk fits even if `MaxReqSize` is larger than the capacity.

Stream DMA applies this capacity check on its timing path. Its
`atomic_noncaching` bypass path can transfer smaller stream beats and
retains that behavior. The shared DMA read FIFO fills available space on
the bypass path, including space smaller than a cache line, so an otherwise
valid consumer chunk can make progress after a partial consumption.
Timing requests retain their existing splitting and request-slot limits.

## Running the SALAM regression suites

Install matching Clang/LLVM, the ARM bare-metal compiler and libraries
(`gcc-arm-none-eabi`, `libnewlib-arm-none-eabi`, and
`libstdc++-arm-none-eabi-newlib`), and the normal gem5 dependencies. Tests
build fresh checked workloads in temporary directories.

```bash
cd tests
./main.py list gem5/salam --include-tags salam -q --suites \
    > /tmp/salam-suites.txt
./main.py run gem5/salam --include-tags salam -j 8
python3 gem5/salam/check_salam_results.py /tmp/salam-suites.txt \
    testing-results/results.xml gem5/salam
```

The checker rejects missing suites, skipped required tests, failures, and
zero-test results. The suite includes source-import isolation with a
negative control, checked BFS through both launchers, exact parity,
functional-unit scheduling, clock changes, and statistics retry/reset
coverage. Remap-profile tests use a separate tag and profile; see the
hardware-profile section above. Use `--skip-build` only with an existing
binary and matching build provenance.

The `profile-singleton` and `profile-reordered` tags check
ownership with separate build profiles. Run each tag in a separate test
process, then rebuild the default profile before ordinary SALAM suites.

## Hardware Profiles

`--with-salam` uses the default hardware profile
`src/salam/hw_profiles/default/salam-hw-config.yml`. It embeds ordered
functional-unit and instruction parameter records as `m5._salam_profile`,
generated under the variant build directory. Paths in the profile are
relative to that file. The build does not modify the profile YAML, and the
simulator does not read that YAML at runtime.

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

`functional_unit_order` is the collection order. It must list every
non-hidden functional-unit directory once. Hidden names and files that
are not directories are ignored.

The default hardware profile is a 40 nm, 5 ns description.
The workload `Profile` field selects the `hw_config` mapping that
supplies per-kernel `runtime_cycles` through `CycleCounts`. That
workload field is not the build-time hardware profile.

### Functional unit construction

The embedded `gem5.salam.cluster` construction gives each accelerator the
selected profile's units and instructions through
`FunctionalUnits.from_profile()` and `InstConfig.from_profile()`. Each call
creates independent children. Bare `FunctionalUnits()` and `InstConfig()`
construction uses the same profile defaults. All entries use the fixed
`FunctionalUnit` and `InstructionConfig` SimObject types. Profiles describe
instances rather than new Python or C++ classes.

Each entry is owned by its named child path, such as
`hw_interface.functional_units.integer_adder`. The typed
`functional_unit_list` and `inst_list` parameters reference those same
children in profile order. Named scalar overrides and whole-child
replacement must happen before parameter unproxying and instantiation.
Collection cloning preserves independent children, and
`FunctionalUnits.characterization()` returns an independent copy of the
embedded characterization records.

Use common entry types with the complete profile parameters when replacing
children. Per-entry class names such as `IntegerAdder` and `Add` are not
exported. Entry names
must be Python identifiers other than keywords, cannot start with an
underscore, and cannot conflict with collection parameters, methods, or
proxy selectors such as `any`, `all`, `search_self`, or `search_up`.
Invalid aliases identify the
selected profile and collection.

A profile may add, remove or rename units without editing handwritten
code. Every instruction's `functional_unit` must be the `enum_value` of a
unit in the profile, or 0 for none; gem5 rejects an instruction in the IR
that names any other value.

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

For example, to run `fsub` on the example unit, copy
`src/salam/hw_profiles/default/` and, in the copy:

1. copy `src/salam/hw_profiles/examples/float_trig_sine/` into
   `functional_units/`;
2. remove `fsub` from the `instructions` list in
   `functional_units/double_adder/double_adder.yml`;
3. replace `- none` with `- fsub` in
   `functional_units/float_trig_sine/float_trig_sine.yml`;
4. append `float_trig_sine` to `functional_unit_order` in
   `salam-hw-config.yml`.

Build with `--salam-hw-config` set to the copy's
`salam-hw-config.yml`. Each `fsub` then needs a `float_trig_sine`
instance (enum 17); the `salam-kernel-fsub-remap` test (tag
`fu-remap`) builds this profile.

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

Latency comes only from `CycleCounts` and the workload `hw_config`
overrides. `cycles` and `stages` describe the unit, and the
instruction entry's `runtime_cycles` and `functional_unit_limit` are
retained as metadata on the common entry objects; none of them changes
when an operation completes or how many issue. Legacy FU power and area
parameters are also metadata; runtime PPA coefficients come from the
characterization records.

The supported terminators `br`, `ret` and `switch_inst` transfer control
without a separate modeled latency. Their `CycleCounts` values and workload
`runtime_cycles` overrides must be zero; nonzero settings are rejected.
Other LLVM terminators, including `indirectbr`, `invoke`, `resume` and
`unreachable`, remain unsupported and are rejected when the IR is read.
A call completes on the caller's compute pass after the callee returns,
plus its configured call latency in further passes. Thus a call latency of
N completes N+1 compute edges after the callee return, including one edge
for N=0. Completion, dependency notification and call accounting occur once
through the caller's compute queue.
This model does not simulate a branch predictor or a control-flow pipeline.

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
        functional_unit_limit: 0
        opcode_num: 13
        runtime_cycles: 1
```

- The key names an `InstructionConfig` child, such as `inst_config.add`,
  which `InstConfig.from_profile()` constructs. Adding an entry does not
  add execution support. Execution is in
  `LLVMInterface::createInstruction()` and
  `src/salam/LLVMRead/instruction.cc`.
- `opcode_num` is the LLVM `Instruction::getOpcode()` value from
  `llvm/IR/Instruction.def`. `createInstruction()` matches it to find
  the entry's functional unit. It cannot define a new operation.
- `functional_unit` is a functional unit's `enum_value`; 0 means none.
  The generator sets this value in memory from the `enum_value` of the
  unit whose `instructions` list contains the instruction. It does not
  modify `inst_list.yml`. An instruction no unit lists keeps its
  configured value. The generator rejects an instruction listed by two
  units. To remap an instruction, move its name from one unit's list
  to another's.
- `functional_unit_limit` is retained as instruction metadata. No
  scheduling code reads it; the unit's `limit` sets its instances.
- `runtime_cycles` is retained as instruction metadata. It does not set
  execution latency. Execution uses
  `CycleCounts`, with overrides from the workload `hw_config` mapping
  that `Profile` selects. Instructions without an override keep the
  `CycleCounts` defaults.

### Exporting profile data

`util/SALAM-tools/hw_generator/HWProfileGenerator.py` can write the
same embedded profile-data module outside the repository. It takes
`--hw-config` and `--output-dir`, and it refuses to write under `src/`.
A normal `--with-salam` build does not need this tool.

## Offline CACTI characterization

`util/SALAM-tools/cacti-SALAM/import_cacti.py` converts supported, retained
raw CACTI output into the characterization format consumed by existing
SALAM memory objects. It validates source identity, units, capacity,
organization, and ports. The simulator never runs CACTI. See the
[import and binding examples](PPA_and_Validation.md) for supported formats
and physical-model limitations.

The older **cacti-SALAM** setup/run scripts below are offline batch helpers
for accelerator YAML configurations. Their summary CSV is not itself an
accepted runtime characterization record.

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
`util/SALAM-tools/cacti-SALAM/results/SALAM-out.csv`. Retain the underlying
raw CACTI output and its producing inputs. Use the validated importer and
an explicit memory `Characterization` reference to select coefficients;
no source-code formula changes are required.

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
