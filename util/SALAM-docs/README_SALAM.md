# gem5-SALAM #

gem5-SALAM (System Architecture for LLVM-based Accelerator Modeling), is a novel system architecture designed to enable LLVM-based modeling and simulation of custom hardware accelerators.

# Requirements

- gem5 dependencies
- LLVM 11–20 (see `SUPPORTED_LLVM_VERSIONS` in `SConstruct`)
- Frontend LLVM compiler for preferred development language (eg. clang for C)
- ARM bare-metal cross-compiler (`gcc-arm-none-eabi`) to build SALAM benchmarks / host ELFs

# gem5-SALAM Setup

## All Required Dependencies for gem5-SALAM (Ubuntu 20.04)

```bash
sudo apt install build-essential git m4 scons zlib1g zlib1g-dev \
    libprotobuf-dev protobuf-compiler libprotoc-dev libgoogle-perftools-dev \
    python3-dev python-is-python3 libboost-all-dev pkg-config
```

## LLVM/Clang Setup

For a quick start, one can simply run the following to install LLVM and Clang on Ubuntu 20.04.
```bash
sudo apt install llvm-12 llvm-12-tools clang-12
```
Ensure `llvm-config` and `clang` are on your `PATH`. If your package manager installs versioned binaries (for example `llvm-config-12`), point `LLVM_CONFIG` at the matching `llvm-config` or create symlinks so the unversioned names resolve.

Alternatively, you can install a supported LLVM version via your system package manager or build from source found at https://github.com/llvm/llvm-project.

## ARM Cross-Compiler (for benchmarks)

To compile SALAM host programs and related bare-metal support code:

```bash
sudo apt-get install gcc-multilib gcc-arm-none-eabi
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

For more information regarding the binary types, and other build information refer to the gem5 build documentation [here](http://learning.gem5.org/book/part1/building.html).

# Using gem5-SALAM

To use gem5-SALAM you need to define the computation model of your accelerator in your language of choice, and compile it to LLVM IR. Any control and dataflow graph optimization (eg. loop unrolling) should be handled by the compiler. You can construct accelerators by associating their LLVM IR with an LLVMInterface and connecting it to the desired CommInterface in the gem5 memory map.

Below are some resources in the gem5-SALAM directory that can be used when getting started:

- Examples for system-level configuration can be found in **configs/SALAM/HWAcc.py**.
- The in-tree accelerator example is **BFS** under **configs/example/gem5_library/salam-benchmarks/** (`src/bfs`). Additional SALAM benchmarks will be made available later through [gem5 Resources](https://resources.gem5.org).
- You can also add your own benchmarks. A detailed walkthrough is in **util/SALAM-docs/Building_and_Integrating_Accelerators.md**.

## Quickstart: build and run BFS

The BFS example under **configs/example/gem5_library/salam-benchmarks** shows how to interface with the gem5-SALAM simulation objects.

`run_system.sh` requires **M5_PATH** and **ACC_BENCH_PATH**. Point `ACC_BENCH_PATH` at the buildable **`src/`** tree (it has Makefiles). The sibling **`workloads/`** tree is a precompiled snapshot without Makefiles and is not the default path when `BUILD=True` (the script default). Invoke the script with `bash` (its shebang is not on the first line). Paths passed to `--bench-path` are relative to `ACC_BENCH_PATH`.

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src

cd $M5_PATH
scons build/ARM/gem5.opt --with-salam -j`nproc`

# Optional: build the workload yourself first
# cd $ACC_BENCH_PATH/bfs && make

bash $M5_PATH/util/SALAM-tools/run_system.sh --bench bfs --bench-path bfs
```

With `BUILD=True` (the default), `run_system.sh` runs `make all` in `$ACC_BENCH_PATH/bfs` before launching gem5. To inspect the gem5 command line it constructs, see the **RUN_SCRIPT** variable in the shell file.

## Hardware Profiles

### Default hardware profile

gem5-SALAM ships with a checked-in **default hardware model** (generated C++/Python under **src/salam/HWModeling/** and **src/salam/FunctionalUnits.py**). It targets a **40nm technology node** with a **5ns cycle-time** budget. Rebuild gem5 with `--with-salam` and use `run_system.sh` as usual; you do not need to regenerate this model to run the in-tree BFS example.

The **YAML sources** used to regenerate that model live with the workload, not under `src/salam/HWModeling/`. The BFS example under **configs/example/gem5_library/salam-benchmarks/src/bfs** includes them at **configs/hw_interface/** (`functional_units/40nm_model/5ns/default_profile/`, plus `instructions/inst_list.yml`).

### Where profile numbers come from

Each functional-unit YAML file has two parts:

* **`parameters`** — microarchitectural description used by **HWProfileGenerator**: pipeline `stages` and `cycles`, `enum_value`, supported datatypes, which LLVM instructions bind to the unit, and scheduling `limit`.
* **`power_model`** — device-level characterization at the chosen technology node (`40nm_model` in the default profile): `area`, `leakage_power`, `internal_power`, `switch_power`, `dynamic_power`, `dynamic_energy`, `path_delay`, and a target `latency` (ns). These values are stored per functional unit in the YAML and compiled into the SALAM SimObjects under **src/salam/HWModeling/** and **src/salam/FunctionalUnits.py**.

The directory name **`5ns`** under `functional_units/40nm_model/` is the profile's target cycle-time bucket (matching the default `power_model.latency` values in the YAML). **`default_profile/`** is the set of units **HWProfileGenerator** reads when you regenerate.

Separately, each workload **config.yml** `hw_config` section supplies per-kernel **`runtime_cycles`** at simulation time via `AccConfig()`. Those workload-specific instruction latencies are not regenerated by **HWProfileGenerator**; update them manually if you change functional-unit bindings or timing after regenerating.

To compute power from your own or profile coefficients during a run, extend **`LLVMInterface::printResults()`** in **src/salam/llvm_interface.cc** (that is where the current end-of-run power/area estimates live).

### Customizing or regenerating a profile

To customize functional-unit or instruction timing models, edit the YAML hardware profile and rerun **util/SALAM-tools/hw_generator**. To see how a custom hardware profile is specified, refer to the in-tree BFS example:

**configs/example/gem5_library/salam-benchmarks/src/bfs/configs/hw_interface/**

* `instructions/inst_list.yml` — master instruction list (the generator updates functional-unit mappings here)
* `functional_units/40nm_model/5ns/default_profile/*/*.yml` — functional units read by **HWProfileGenerator**
* `functional_units/40nm_model/5ns/additional_units/float_trig_sine/` — example optional functional unit (see below)

### Adding a custom functional unit

The `float_trig_sine` YAML under `additional_units/` shows how to define an optional functional unit: alias, pipeline stages, cycles, `enum_value`, supported datatypes, instruction bindings, and a CACTI-style power model. **HWProfileGenerator** only processes YAML files under `default_profile/`. The `additional_units/` tree is not read automatically; it is included as a reference for how to structure custom units.

To include a custom unit when regenerating:

1. Copy the unit directory from `additional_units/` into `default_profile/` (for example, `default_profile/float_trig_sine/float_trig_sine.yml`).
2. Bind LLVM instructions to the unit in `inst_list.yml` (and update the workload **config.yml** `hw_config` section if per-kernel `runtime_cycles` change).
3. Run **HWProfileGenerator**, rebuild gem5 with `--with-salam`, and rerun the workload.

### Regenerating from YAML

`--bench-path` is the workload directory under `ACC_BENCH_PATH` (defaults to the same name as `-b` / `--bench`). For in-tree BFS with `ACC_BENCH_PATH=.../salam-benchmarks/src`, that is `bfs` → `$ACC_BENCH_PATH/bfs`.

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src
python3 util/SALAM-tools/hw_generator/HWProfileGenerator.py -b bfs
```

Then rebuild gem5 (`scons build/ARM/gem5.opt --with-salam`) before running simulations.

_Note that the default checked-in SALAM hardware profile uses the floating-point functional unit mapping needed by the provided 32-bit ARM examples. Custom profiles should define their desired floating-point units and latencies in YAML before regenerating the hardware model. Future work may make floating-point functional unit selection more directly type-aware from the LLVM instruction precision._

## Power Modeling using cacti-SALAM

**cacti-SALAM** (`util/SALAM-tools/cacti-SALAM`) is an **offline helper** for running CACTI on accelerator YAML configs. It is **not wired into** gem5-SALAM simulation.

```bash
cd util/SALAM-tools/cacti-SALAM
./setup_cacti_SALAM.py
```

Create `$ACC_BENCH_PATH/benchmarks.list` with lines of the form:

```
path/to/config.yml <benchmark name> <config name>
```

Then:

```bash
python3 ./run_cacti_SALAM.py --bench-list $ACC_BENCH_PATH/benchmarks.list --delay 1.0
```

Results are written to `util/SALAM-tools/cacti-SALAM/results/SALAM-out.csv`. To use those (or other) coefficients in a simulation power estimate, add code in **`LLVMInterface::printResults()`** in **src/salam/llvm_interface.cc**.

# Resources

## gem5 Documentation

https://www.gem5.org/documentation/

## gem5 Tutorial

The gem5 documentation has a [tutorial for working with gem5](http://learning.gem5.org/book/index.html#) that will help get you started with the basics of creating your own sim objects.

## Building and Integrating Accelerators in gem5-SALAM

We have written a guide that walks through the in-tree BFS example. This will help you get started with creating your own benchmarks and systems. It can be viewed at **util/SALAM-docs/Building_and_Integrating_Accelerators.md**.

## SALAM Object Overview

The overview at **util/SALAM-docs/SALAM_Object_Overview.md** covers what various Sim Objects in gem5-SALAM are and their purpose.

## Full-system OS Simulation ##

Please download the latest version of the Linux Kernel for ARM from the [gem5 ARM kernel page](http://gem5.org/ARM_Kernel).
You will also need the [ARM disk images](http://www.gem5.org/dist/current/arm/) for full system simulation.
Devices operate in the physical memory address space.
