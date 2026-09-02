# gem5-SALAM

gem5-SALAM (System Architecture for LLVM-based Accelerator Modeling), is a novel system architecture designed to enable LLVM-based modeling and simulation of custom hardware accelerators.

# Requirements

- gem5 build dependencies
- LLVM development headers and libraries
- An LLVM frontend for generating accelerator IR (Clang for the in-tree C benchmarks)
- ARM bare-metal cross-compiler (`gcc-arm-none-eabi`) to build SALAM host ELFs

# gem5-SALAM Setup

## Dependencies

Install the host dependencies for gem5 using the
[gem5 build documentation](https://www.gem5.org/documentation/general_docs/building).

SALAM additionally requires:

- LLVM development headers and libraries
- Clang to generate accelerator LLVM IR
- an ARM bare-metal cross compiler (`gcc-arm-none-eabi`) to build SALAM host ELFs

When building with `--with-salam`, `SConstruct` first uses `LLVM_CONFIG` if it points to an existing `llvm-config` executable, then `llvm-config` on `PATH`, and otherwise searches for the highest available `llvm-config-N` where `N` is 11 through 20.

The current port has been verified to build with LLVM/Clang 14 on Ubuntu 22.04 and LLVM/Clang 18 on Ubuntu 24.04.

To select a particular LLVM installation, set `LLVM_CONFIG` to its
`llvm-config` executable. For compatibility, it is recommended to generate accelerator LLVM IR with a Clang from the same LLVM major version.

On Ubuntu, install the SALAM-specific dependencies:

```bash
sudo apt install llvm-dev clang gcc-arm-none-eabi
```

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

- Examples for system-level configuration can be found in **configs/SALAM/HWAcc.py**.
- The in-tree accelerator example is **BFS** under **configs/example/gem5_library/salam-benchmarks/** (`src/bfs`). Additional SALAM benchmarks will be made available later through [gem5 Resources](https://resources.gem5.org).
- You can also add your own benchmarks. A detailed walkthrough is in **util/SALAM-docs/Building_and_Integrating_Accelerators.md**.

## Quickstart: build and run BFS

The BFS example under **configs/example/gem5_library/salam-benchmarks** shows how to interface with the gem5-SALAM simulation objects.

`run_system.sh` requires **M5_PATH** and **ACC_BENCH_PATH**. Point `ACC_BENCH_PATH` at the buildable `src/` tree (it has Makefiles). The sibling `workloads/` tree is a precompiled snapshot without Makefiles and is not the default path when `BUILD=True` (the script default).

`--bench` should be a simple identifier such as `bfs`; it names the generated SALAM configuration and is also passed to the simulation as the accelerator benchmark name. `--bench-path` separately specifies the workload directory relative to `ACC_BENCH_PATH`; if omitted, it defaults to the `--bench` value.

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

### Default hardware profile

gem5-SALAM ships with a checked-in **default hardware model** (generated C++/Python under **src/salam/HWModeling/** and **src/salam/FunctionalUnits.py**). It targets a **40nm technology node** with a **5ns cycle-time** budget. Rebuild gem5 with `--with-salam` and use `run_system.sh` as usual; you do not need to regenerate this model to run the in-tree BFS example.

The **YAML sources** used to regenerate that model live with the workload, not under `src/salam/HWModeling/`. The BFS example under **configs/example/gem5_library/salam-benchmarks/src/bfs** includes them at **configs/hw_interface/** (`functional_units/40nm_model/5ns/default_profile/`, plus `instructions/inst_list.yml`).

### Where profile numbers come from

Each functional-unit YAML file has two parts:

- `parameters` — microarchitectural description used by **HWProfileGenerator**: pipeline `stages` and `cycles`, `enum_value`, supported datatypes, which LLVM instructions bind to the unit, and scheduling `limit`.
- `power_model` — device-level characterization at the chosen technology node (`40nm_model` in the default profile): `area`, `leakage_power`, `internal_power`, `switch_power`, `dynamic_power`, `dynamic_energy`, `path_delay`, and a target `latency` (ns). These values are stored per functional unit in the YAML and compiled into the SALAM SimObjects under **src/salam/HWModeling/** and **src/salam/FunctionalUnits.py**.

The directory name `5ns` under `functional_units/40nm_model/` is the profile's target cycle-time bucket (matching the default `power_model.latency` values in the YAML). `default_profile/` is the set of units **HWProfileGenerator** reads when you regenerate.

Separately, each workload **config.yml** `hw_config` section supplies per-kernel `runtime_cycles` at simulation time via `AccConfig()`. Those workload-specific instruction latencies are not regenerated by **HWProfileGenerator**; update them manually if you change functional-unit bindings or timing after regenerating.

To compute power from your own or profile coefficients during a run, extend `LLVMInterface::printResults()` in **src/salam/llvm_interface.cc** (that is where the current end-of-run power/area estimates live).

### Customizing or regenerating a profile

To customize functional-unit or instruction timing models, edit the YAML hardware profile and rerun **util/SALAM-tools/hw_generator**. To see how a custom hardware profile is specified, refer to the in-tree BFS example:

**configs/example/gem5_library/salam-benchmarks/src/bfs/configs/hw_interface/**

- `instructions/inst_list.yml` — master instruction list (the generator updates its `functional_unit` fields from the FU YAML `instructions` lists)
- `functional_units/40nm_model/5ns/default_profile/*/*.yml` — functional units read by **HWProfileGenerator**
- `functional_units/40nm_model/5ns/additional_units/float_trig_sine/` — example optional functional unit (see below)

### Adding a custom functional unit

SALAM maps LLVM instructions to functional-unit (FU) models. The
`instructions` list in each FU YAML specifies which existing LLVM
instructions are associated with that unit. During regeneration,
**HWProfileGenerator** uses these lists to update the corresponding
`functional_unit` fields in `inst_list.yml`.

The `additional_units/` directory contains example FU definitions that are
not part of the default profile. To include one when regenerating:

1. Copy its directory from `additional_units/` into `default_profile/`.
2. Add the LLVM instruction names modeled by that unit to its `instructions`
  list, removing them from any previous FU's list.
3. If this introduces a new generated SimObject class, add its class name to
   the `sim_objects` list for `FunctionalUnits.py` in `src/salam/SConscript`.
4. Run **HWProfileGenerator**, rebuild gem5 with `--with-salam`, and rerun the
  workload.

Existing `opcode_num` values are part of SALAM's checked-in instruction
configuration; they are not user-assigned IDs for defining new LLVM
instructions. When adding or remapping a functional unit, list an
existing supported instruction in the FU YAML `instructions` field and
let the generator retain that instruction's existing configuration.
Supporting a new LLVM instruction requires extending the instruction
model rather than inventing a new `opcode_num`.

The current scheduler does not enforce FU occupancy when launching
instructions, so `functional_unit_limit` is not an effective
resource-constrained performance-modeling parameter on this branch.

### Regenerating the hardware model from YAML

After modifying the instruction or functional-unit YAML files, run
**HWProfileGenerator** to regenerate the SALAM hardware model. `--bench-path`
identifies the workload directory under `ACC_BENCH_PATH` and defaults to the
`-b` / `--bench` value. For in-tree BFS with
`ACC_BENCH_PATH=.../salam-benchmarks/src`, that is
`bfs` → `$ACC_BENCH_PATH/bfs`.

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src
python3 util/SALAM-tools/hw_generator/HWProfileGenerator.py -b bfs
```

Then rebuild gem5 (`scons build/ARM/gem5.opt --with-salam`) before running simulations.

*Note that the default checked-in SALAM hardware profile uses the floating-point functional unit mapping needed by the provided 32-bit ARM examples. Custom profiles should define their desired floating-point units and latencies in YAML before regenerating the hardware model. The current generator does not automatically select floating-point functional units based on LLVM instruction precision.*

## Power Modeling using cacti-SALAM

**cacti-SALAM** (`util/SALAM-tools/cacti-SALAM`) is an **offline helper** for running CACTI on accelerator YAML configs. It is **not wired into** gem5-SALAM simulation.

On x86_64 Ubuntu, its 32-bit build additionally requires:

```bash
sudo apt install gcc-multilib g++-multilib
```

```bash
cd util/SALAM-tools/cacti-SALAM
./setup_cacti_SALAM.py
```

Create `$ACC_BENCH_PATH/benchmarks.list` with three whitespace-separated
fields per line:

```
path/to/config.yml <benchmark-name> <config-name>
```

The first field identifies the YAML configuration. The remaining two fields
are labels written to the `Benchmark` and `Config` columns of
`SALAM-out.csv`.

Then:

```bash
python3 ./run_cacti_SALAM.py --bench-list $ACC_BENCH_PATH/benchmarks.list --delay 1.0
```

Results are written to `util/SALAM-tools/cacti-SALAM/results/SALAM-out.csv`. To use those (or other) coefficients in a simulation power estimate, add code in `LLVMInterface::printResults()` in **src/salam/llvm_interface.cc**.

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
