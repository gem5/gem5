# gem5-SALAM #

gem5-SALAM (System Architecture for LLVM-based Accelerator Modeling), is a novel system architecture designed to enable LLVM-based modeling and simulation of custom hardware accelerators.

# Requirements

- gem5 dependencies
- LLVM-9 or newer
- Frontend LLVM compiler for preferred development language (eg. clang for C)

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
After installing these specific libraries, simply run the [update alternatives](https://github.com/TeCSAR-UNCC/gem5-SALAM/blob/main/docs/update-alternatives.sh) script in docs/.

Alternatively, you can install the latest version of LLVM via your system package manager or build from source found at https://github.com/llvm/llvm-project.

# Building gem5-SALAM

When building gem5-SALAM, there are multiple different binary types that can be created. Just like in gem5 the options are debug, opt, fast, prof, and perf. We recommend that users either use the opt or debug builds, as these are the build types we develop and test on.

Below are the bash commands you would use to build the opt or debug binary.

```bash
scons build/ARM/gem5.opt --with-salam -j`nproc`
```

```bash
scons build/ARM/gem5.debug --with-salam -j`nproc`
```

For more information regarding the binary types, and other build information refer to the gem5 build documentation [here](http://learning.gem5.org/book/part1/building.html).

# Building with docker
You can use the Dockerfile given in the `docker/` directory to build the project and run the benchmarks. To build the project use the following command:
```bash
docker build . --file docker/Dockerfile --build-arg BUILD_TYPE="opt"
```

The `BUILD_TYPE` argument sets the the building option for the project and can be `opt` or `debug`.

# Using gem5-SALAM

To use gem5-SALAM you need to define the computation model of you accelerator in your language of choice, and compile it to LLVM IR. Any control and dataflow graph optimization (eg. loop unrolling) should be handled by the compiler. You can construct accelerators by associating their LLVM IR with an LLVMInterface and connecting it to the desired CommInterface in the gem5 memory map.

Below are some resources in the gem5-SALAM directory that can be used when getting started:

- Examples for system-level configuration can be found in **configs/SALAM/HWAcc.py**.
- Accelerator benchmarks and examples can be found in **gem5-resources**.

## System Validation Examples

The system validation examples under **configs/example/gem5-library/salam-benchmarks** are good examples for how you interface with the gem5-SALAM simulation objects.

In order to use the system validation benchmarks, it is required to have the ARM GCC cross-compiler installed. If you didn't already install it when you setup the dependencies, you can install it in Ubuntu by running the below command:

```bash
sudo apt-get install gcc-multilib gcc-arm-none-eabi
```

**run_system.sh** requires environment variables named **M5_PATH** and **ACC_BENCH_PATH** to be set. You will want to point them to your gem5 and benchmark root paths (respectively) as shown below.

```bash
export M5_PATH=/path/to/gem5 root
```

```bash
export ACC_BENCH_PATH=/path/to/benchmarks root
```

All paths passed at runtime as arguments would be relative to this benchmark root.

Next, compile your desired example.

```bash
cd $ACC_BENCH_PATH/[benchmark path relative to ACC_BENCH_PATH]
make
```
Alternatively, you can set the BUILD option to True in `run_system.sh`.

Finally, you can run any of the benchmarks you have compiled by running the run system script.

```bash
$M5_PATH/util/SALAM-tools/run_system.sh --bench [benchmark name] --bench-path [benchmark path relative to ACC_BENCH_PATH]
```

For instance, for bfs you would run:

```bash
$M5_PATH/util/SALAM-tools/run_system.sh --bench bfs --bench-path bfs
```

If you would like to see the gem5-SALAM command created by the shell file you would just need to inspect the **RUN_SCRIPT** variable in the shell file.

## Hardware Profiles

### Default hardware profile

gem5-SALAM ships with a checked-in **default hardware profile** under **src/salam/HWModeling/**. It targets a **40nm technology node** with a **5ns cycle-time** budget and is organized in YAML as `functional_units/40nm_model/5ns/default_profile/`. That profile covers the functional units and instruction mappings needed to run all benchmarks in the sys_validation SALAM suite. Rebuild gem5 with `--with-salam` and use `run_system.sh` as usual; you do not need to regenerate this model to run those benchmarks.

The BFS example under **configs/example/gem5_library/salam-benchmarks/src/bfs** includes a copy of that profile layout under **configs/hw_interface/** so you can see how the default was specified and regenerate locally if you change it.

### Where profile numbers come from

Each functional-unit YAML file has two parts:

* **`parameters`** — microarchitectural description used by **HWProfileGenerator**: pipeline `stages` and `cycles`, `enum_value`, supported datatypes, which LLVM instructions bind to the unit, and scheduling `limit`.
* **`power_model`** — device-level characterization at the chosen technology node (`40nm_model` in the default profile): `area`, `leakage_power`, `internal_power`, `switch_power`, `dynamic_power`, `dynamic_energy`, `path_delay`, and a target `latency` (ns). These values are stored per functional unit in the YAML and compiled into the SALAM SimObjects under **src/salam/HWModeling/** and **src/salam/FunctionalUnits.py**.

The directory name **`5ns`** under `functional_units/40nm_model/` is the profile's target cycle-time bucket (matching the default `power_model.latency` values in the YAML). **`default_profile/`** is the set of units **HWProfileGenerator** reads when you regenerate.

Separately, each workload **config.yml** `hw_config` section supplies per-kernel **`runtime_cycles`** at simulation time via `AccConfig()`. Those workload-specific instruction latencies are not regenerated by **HWProfileGenerator**; update them manually if you change functional-unit bindings or timing after regenerating.

Power and area for accelerator **scratchpad memories** (Vars, SPMs) are modeled separately through **cacti-SALAM** (see below), not through the functional-unit YAML `power_model` block.

### Customizing or regenerating a profile

To customize functional-unit or instruction timing models, edit the YAML hardware profile and rerun **util/SALAM-tools/hw_generator**. Profile inputs for the BFS example live at:

**configs/example/gem5_library/salam-benchmarks/src/bfs/configs/hw_interface/**

* `instructions/inst_list.yml` — master instruction list (the generator updates functional-unit mappings here)
* `functional_units/40nm_model/5ns/default_profile/*/*.yml` — functional units read by **HWProfileGenerator**
* `functional_units/40nm_model/5ns/additional_units/float_trig_sine/` — example optional functional unit (see below)

Benchmarks in a separate benchmarks repository use the same layout under **sys_validation/\<bench\>/configs/hw_interface/**.

### Adding a custom functional unit

The `float_trig_sine` YAML under `additional_units/` shows how to define an optional functional unit: alias, pipeline stages, cycles, `enum_value`, supported datatypes, instruction bindings, and a CACTI-style power model. **HWProfileGenerator** only processes YAML files under `default_profile/`. The `additional_units/` tree is not read automatically; it is included as a reference for how to structure custom units.

To include a custom unit when regenerating:

1. Copy the unit directory from `additional_units/` into `default_profile/` (for example, `default_profile/float_trig_sine/float_trig_sine.yml`).
2. Bind LLVM instructions to the unit in `inst_list.yml` (and update the workload **config.yml** `hw_config` section if per-kernel `runtime_cycles` change).
3. Run **HWProfileGenerator**, rebuild gem5 with `--with-salam`, and rerun the workload.

### Regenerating from YAML

```bash
export M5_PATH=/path/to/gem5
export ACC_BENCH_PATH=$M5_PATH/configs/example/gem5_library/salam-benchmarks/src
python3 util/SALAM-tools/hw_generator/HWProfileGenerator.py -b bfs --bench-path bfs
```

For benchmarks under `sys_validation/<bench>/` relative to `ACC_BENCH_PATH`:

```bash
export ACC_BENCH_PATH=/path/to/benchmarks
python3 util/SALAM-tools/hw_generator/HWProfileGenerator.py -b bfs
```

Then rebuild gem5 (`scons build/ARM/gem5.opt --with-salam`) before running simulations.

_Note that the default checked-in SALAM hardware profile uses the floating-point functional unit mapping needed by the provided 32-bit ARM examples. Custom profiles should define their desired floating-point units and latencies in YAML before regenerating the hardware model. Future work may make floating-point functional unit selection more directly type-aware from the LLVM instruction precision._

## Power Modeling using cacti-SALAM

The cacti-SALAM toolchain is a mini-suite of Python scripts to drive CACTI analyses on SALAM scratchpad memories based on YAML accelerator config, aggregating power/area/delay results.

Start by running the setup script:

```bash
cd util/SALAM-tools/cacti-SALAM
./setup_cacti_salam.py
```

Next, in `$ACC_BENCH_PATH/benchmarks.list`, prepare a list of lines with the following fields:

```
path/to/config.yml <benchmark name> <config name>
```

Finally, run cacti-SALAM using the following script to generate the lookup table:

```bash
python3 ./run_cacti_salam.py --bench-list $ACC_BENCH_PATH/benchmarks.list --delay 1.0
```

Check `tools/cacti-SALAM/results/SALAM-out.csv` for the consolidated table of `Benchmark,Config,Acc,…<CACTI columns>`, which will be used by the simulation to generate power/area/delay results.

# Resources

## gem5 Documentation

https://www.gem5.org/documentation/

## gem5 Tutorial

The gem5 documentation has a [tutorial for working with gem5](http://learning.gem5.org/book/index.html#) that will help get you started with the basics of creating your own sim objects.

## Building and Integrating Accelerators in gem5-SALAM

We have written a guide on how to create the GEMM system validation example. This will help you get started with creating your own benchmarks and systems. It can be viewed [here](https://github.com/TeCSAR-UNCC/gem5-SALAM/blob/master/docs/Building_and_Integrating_Accelerators.md).

## SALAM Object Overview

The [SALAM Object Overview](https://github.com/TeCSAR-UNCC/gem5-SALAM/blob/master/docs/SALAM_Object_Overview.md) covers what various Sim Objects in gem5-SALAM are and their purpose.

## Full-system OS Simulation ##

Please download the latest version of the Linux Kernel for ARM from the [gem5 ARM kernel page](http://gem5.org/ARM_Kernel).
You will also need the [ARM disk images](http://www.gem5.org/dist/current/arm/) for full system simulation.
Devices operate in the physical memory address space.
