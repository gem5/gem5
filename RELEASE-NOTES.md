# Version 23.1

gem5 Version 23.1 is our first release where the development has been on GitHub.
During this release, there have been 362 pull requests merged which comprise 416 commits with 51 unique contributors.

## Significant API and user-facing changes

### The gem5 build can is now configured with `kconfig`

- Most gem5 builds without customized options (excluding double dash options) (e.g. , build/X86/gem5.opt) are backwards compatible and require no changes to your current workflows.
- All of the default builds in `build_opts` are unchanged and still available.
- However, if you want to specialize your build. For example, use customized ruby protocol. The command `scons PROTOCOL=<PROTOCAL_NAME> build/ALL/gem5.opt` will not work anymore. you now have to use `scons <kconfig command>` to update the ruby protocol as example. The double dash options (`--without-tcmalloc`, `--with-asan` and so on) are still continue to work as normal.
- For more details refer to the documentation here: [kconfig documentation](https://www.gem5.org/documentation/general_docs/kconfig_build_system/)

### Standard library improvements

#### `WorkloadResource` added to resource specialization

- The `Workload` and `CustomWorkload` classes are now deprecated. They have been transformed into wrappers for the `obtain_resource` and `WorkloadResource` classes in `resource.py`, respectively.
- Code utilizing the older API will continue to function as expected but will trigger a warning message. To update code using the `Workload` class, change the call from `Workload(id='resource_id', resource_version='1.0.0')` to `obtain_resource(id='resource_id', resource_version='1.0.0')`. Similarly, to update code using the `CustomWorkload` class, change the call from `CustomWorkload(function=func, parameters=params)` to `WorkloadResource(function=func, parameters=params)`.
- Workload resources in gem5 can now be directly acquired using the `obtain_resource` function, just like other resources.

#### Introducing Suites

Suites is a new category of resource being introduced in gem5. Documentation of suites can be found here: [suite documentation](https://www.gem5.org/documentation/gem5-stdlib/suites).

#### Other API changes

- All resource object now have their own `id` and `category`. Each resource class has its own `__str__()` function which return its information in the form of **category(id, version)** like **BinaryResource(id='riscv-hello', resource_version='1.0.0')**.
- Users can use GEM5_RESOURCE_JSON  and GEM5_RESOURCE_JSON_APPEND env variables to overwrite all the data sources with the provided JSON and append a JSON file to all the data source respectively. More information can be found [here](https://www.gem5.org/documentation/gem5-stdlib/using-local-resources).

### Other user-facing changes

- Added support for clang 15 and clang 16
- gem5 no longer supports building on Ubuntu 18.04
- GCC 7, GCC 9, and clang 6 are no longer supported
- Two `DRAMInterface` stats have changed names (`bytesRead` and `bytesWritten`). For instance, `board.memory.mem_ctrl.dram.bytesRead` and `board.memory.mem_ctrl.dram.bytesWritten`. These are changed to `dramBytesRead` and `dramBytesWritten` so they don't collide with the stat with the same name in `AbstractMemory`.
- The stats for `NVMInterface` (`bytesRead` and `bytesWritten`) have been change to `nvmBytesRead` and `nvmBytesWritten` as well.

## Full-system GPU model improvements

- Support for up to latest ROCm 5.7.1.
- Various changes to enable PyTorch/TensorFlow simulations.
- New packer disk image script containing ROCm 5.4.2, PyTorch 2.0.1, and Tensorflow 2.11.
- GPU instructions can now perform atomics on host addresses.
- The provided configs scripts can now run KVM on more restrictive setups.
- Add support to checkpoint and restore between kernels in GPUFS, including adding various AQL, HSA Queue, VMID map, MQD attributes, GART translations, and PM4Queues to GPU checkpoints
- move GPU cache recorder code to RubyPort instead of Sequencer/GPUCoalescer to allow checkpointing to occur
- add support for flushing GPU caches, as well as cache cooldown/warmup support, for checkpoints
- Update vega10_kvm.py to add checkpointing instructions

## SE mode GPU model improvements

- started adding support for mmap'ing inputs for GPUSE tests, which reduces their runtime by 8-15% per run

## GPU model improvements

- update GPU VIPER and Coalescer support to ensure correct replacement policy behavior when multiple requests from the same CU are concurrently accessing the same line
- fix bug with GPU VIPER to resolve a race conflict for loads that bypass the TCP (L1D$)
- fix bug with MRU replacement policy updates in GPU SQC (I$)
- update GPU and Ruby debug prints to resolve various small errors
- Add configurable GPU L1,L2 num banks and L2 latencies
- Add decodings for new MI100 VOP2 insts
- Add GPU GLC Atomic Resource Constraints to better model how atomic resources are shared at GPU TCC (L2$)
- Update GPU tester to work with both requests that bypass all caches (SLC) and requests that bypass only the TCP (L1D$)
- Fixes for how write mask works for GPU WB L2 caches
- Added support for WB and WT GPU atomics
- Added configurable support to better model the latency of GPU atomic requests
- fix GPU's default number of HW barrier/CU to better model amount of concurrency GPU CUs should have

## RISC-V RVV 1.0 implemented

This was a huge undertaking by a large number of people!
Some of these people include Adrià Armejach who pushed it over the finish line, Xuan Hu who pushed the most recent version to gerrit that Adrià picked up,
Jerin Joy who did much of the initial work, and many others who contributed to the implementation including Roger Chang, Hoa Nguyen who put significant effort into testing and reviewing the code.

- Most of the instructions in the 1.0 spec implemented
- Works with both FS and SE mode
- Compatible with Simple CPUs, the O3, and the minor CPU models
- User can specify the width of the vector units
- Future improvements
  - Widening/narrowing instructions are *not* implemented
  - The model for executing memory instructions is not very high performance
  - The statistics are not correct for counting vector instruction execution

## ArmISA changes/improvements

- Architectural support for the following extensions:
* FEAT_TLBIRANGE
* FEAT_FGT
* FEAT_TCR2
* FEAT_SCTLR2

- Arm support for SVE instructions improved
- Fixed some FEAT_SEL2 related issues:
  - [Fix virtual interrupt logic in secure mode](https://github.com/gem5/gem5/pull/584)
  - [Make interrupt masking handle VHE/SEL2 cases](https://github.com/gem5/gem5/pull/430)
- Removed support for Arm Jazelle and ThumbEE
- Implementation of an Arm Capstone Disassembler

## Other notable changes/improvements

- Improvements to the CHI coherence protocol implementation
- Far atomics implemented in CHI
- Ruby now supports using the prefetchers from the classic caches, if the protocol supports it. CHI has been extended to support the classic prefetchers.
- Bug in RISC-V TLB to fixed to correctly count misses and hits
- Added new RISC-V Zcb instructions https://github.com/gem5/gem5/pull/399
- RISC-V can now use a separate binary for the bootloader and kernel in FS mode
- DRAMSys integration updated to latest DRAMSys version (5.0)
- Improved support for RISC-V privilege modes
- Fixed bug in switching CPUs with RISC-V
- CPU branch preditor refactoring to prepare for decoupled front end support
- Perf is now optional when using the KVM CPU model
- Improvements to the gem5-SST bridge including updating to SST 13.0
- Improved formatting of documentation in stdlib
- By default use isort for python imports in style
- Many, many testing improvements during the migration to GitHub actions
- Fixed the elastic trace replaying logic (TraceCPU)

## Known Bugs/Issues

- [RISC-V RVV Bad execution of riscv rvv vss instruction](https://github.com/gem5/gem5/issues/594)
- [RISC-V Vector Extension float32_t bugs/unsupported widening instructions](https://github.com/gem5/gem5/issues/442)
- [Implement AVX xsave/xstor to avoid workaround when checkpointing](https://github.com/gem5/gem5/issues/434)
- [Adding Vector Segmented Loads/Stores to RISC-V V 1.0 implementation](https://github.com/gem5/gem5/issues/382)
- [Integer overflow in AddrRange subset check](https://github.com/gem5/gem5/issues/240)
- [RISCV64 TLB refuses to access upper half of physical address space](https://github.com/gem5/gem5/issues/238)
- [Bug when trying to restore checkpoints in SPARC: “panic: panic condition !pte occurred: Tried to execute unmapped address 0.”](https://github.com/gem5/gem5/issues/197)
- [BaseCache::recvTimingResp can trigger an assertion error from getTarget() due to MSHR in senderState having no targets](https://github.com/gem5/gem5/issues/100)

# Version 23.0.1.0

This minor release incorporates documentation updates, bug fixes, and some minor improvements.

## Documentation updates

* "TESTING.md" has been updated to more accurately reflect our current testing infrastructure.
* "README" has been replaced with "README.md" and includes more up-to-date information on using gem5.
* "CONTRIBUTING.md" has been updated to reflect our migration to GitHub and the changes in policy and proceedures.
* Where needed old references to Gerrit have been removed in favor of GitHub.

## Bug Fixes

* Fixes an assert failure when using ARM which was trigged when `shiftAmt` is 0 for a UQRSH instruction.
* Fixes `name 'fatal' is not defined` being thrown when tracing is off.
* Fixes a bug in ARM in which the TLBIOS instructions were decoded as normal MSR instructions with no effect on the TLBs.
* Fixes invalid `packet_id` value in flit.
* Fixes default CustomMesh for use with Garnet.

## Minor Improvements

* The gem5 resources downloader now outputs more helpful errors in the case of a failure.
* "util/github-runners-vagrant" has been added. This outlines how to setup a GitHub Action's set-hosted runner for gem5.
* The PyUnit tests have been refactored to no longer download large resources during testing.
* Using Perf is now optional when utilizing KVM CPUs.

# Version 23.0.0.1

**[HOTFIX]** Fixes compilation of `GCN3_X86` and `VEGA_X85`.

This hotfix release:

* Removes the use of 'std::random_shuffle'.
This is a deprecated function in C++17 and has been removed in C++20.
* Adds missing 'overrides' in "src/arch/amdgpu/vega/insts/instructions.hh".
* Fixes Linux specific includes, allowing for compilation on non-linux systems.
* Adds a missing include in "src/gpu-compute/dispatcher.cc".

# Version 23.0

This release has approximately 500 contributions from 50 unique contributors.
Below we highlight key gem5 features and improvements in this release.

## Significant API and user-facing changes

### Major renaming of CPU stats

The CPU stats have been renamed.
See <https://gem5.atlassian.net/browse/GEM5-1304> for details.

Now, each stage (fetch, execute, commit) have their own stat group.
Stats that are shared between the different CPU model (O3, Minor, Simple) now have the exact same names.

**Important:** Some stat names were misleading before this change.
With this change, stats with the same names between different CPU models have the same meaning.

### `fs.py` and `se.py` deprecated

These scripts have not been well supported for many gem5 releases.
With gem5 23.0, we have officially deprecated these scripts.
They have been moved into the `deprecated` directory, **but they will be removed in a future release.**
As a replacement, we strongly suggest using the gem5 standard library.
See <https://www.gem5.org/documentation/gem5-stdlib/overview> for more information.

### Renaming of `DEBUG` guard into `GEM5_DEBUG`

Scons no longer defines the `DEBUG` guard in debug builds, so code making using of it should use `GEM5_DEBUG` instead.

### Other API changes

Also, this release:

- Removes deprecated namespaces. Namespace names were updated a couple of releases ago. This release removes the old names.
- Uses `MemberEventWrapper` in favor of `EventWrapper` for instance member functions.
- Adds an extension mechanism to `Packet` and `Request`.
- Sets x86 CPU vendor string to "HygoneGenuine" to better support GLIBC.

## New features and improvements

### Large improvements to gem5 resources and gem5 resources website

We now have a new web portal for the gem5 resources: <https://resources.gem5.org>

This web portal will allow users to browse the resources available (e.g., disk images, kernels, workloads, binaries, simpoints, etc.) to use out-of-the-box with the gem5 standard library.
You can filter based on architecture, resource type, and compatible gem5 versions.

For each resource, there are examples of how to use the resource and pointers to examples using the resource in the gem5 codebase.

More information can be found on gem5's website: <https://www.gem5.org/documentation/general_docs/gem5_resources/>

We will be expanding gem5 resources with more workloads and resources over the course of the next release.
If you would like to contribute to gem5 resources by uploading your own workloads, disk images, etc., please create an issue on GitHub.

In addition to the new gem5 Resources web portal, the gem5 Resources API has been significantly updated and improved.
There are now much simpler functions for getting resources such as `obtain_resource(<name>)` that will download the resource by name and return a reference that can be used (e.g., as a binary in `set_se_workload` function on the board).
As such the generic `Resouce` class has been deprecated and will be removed in a future release.

Resources are now specialized for their particular category.
For example, there is now a `BinaryResource` class which will return if a user specifies a binary resource when using the `obtain_resource` function.
This allow for resource typing and for greater resource specialization.

### Arm ISA improvements

Architectural support for Armv9 [Scalable Matrix extension](https://developer.arm.com/documentation/ddi0616/latest) (FEAT_SME).
The implementation employs a simple renaming scheme for the Za array register in the O3 CPU, so that writes to difference tiles in the register are considered a dependency and are therefore serialized.

The following SVE and SIMD & FP extensions have also been implemented:
* FEAT_F64MM
* FEAT_F32MM
* FEAT_DOTPROD
* FEAT_I8MM

And more generally:

* FEAT_TLBIOS
* FEAT_FLAGM
* FEAT_FLAGM2
* FEAT_RNG
* FEAT_RNG_TRAP
* FEAT_EVT

### Support for DRAMSys

gem5 can now use DRAMSys <https://github.com/tukl-msd/DRAMSys> as a DRAM backend.

### RISC-V improvements

This release:

- Fully implements RISC-V scalar cryptography extensions.
- Fully implement RISC-V rv32.
- Implements PMP lock features.
- Adds general RISC-V improvements to provide better stability.

### Standard library improvements and new components

This release:

- Adds MESI_Three_Level component.
- Supports ELFies and LoopPoint analysis output from Sniper.
- Supports DRAMSys in the stdlib.

## Bugfixes and other small improvements

This release also:

- Removes deprecated python libraries.
- Adds a DDR5 model.
- Adds AMD GPU MI200/gfx90a support.
- Changes building so it no longer "duplicates sources" in build/ which improves support for some IDEs and code analysis. If you still need to duplicate sources you can use the `--duplicate-sources` option to `scons`.
- Enables `--debug-activate=<object name>` to use debug trace for only a single SimObject (the opposite of `--debug-ignore`). See `--debug-help` for more information.
- Adds support to exit the simulation loop based on Arm-PMU events.
- Supports Python 3.11.
- Adds the idea of a CpuCluster to gem5.


# Version 22.1.0.0

This release has 500 contributions from 48 unique contributors and marks our second major release of 2022.
This release incorporates several new features, improvements, and bug fixes for the computer architecture reserach community.

See below for more details!

## New features and improvements

- The gem5 binary can now be compiled to include multiple ISA targets.
A compilation of gem5 which includes all gem5 ISAs can be created using: `scons build/ALL/gem5.opt`.
This will use the Ruby `MESI_Two_Level` cache coherence protocol by default, to use other protocols: `scons build/ALL/gem5.opt PROTOCOL=<other protocol>`.
The classic cache system may continue to be used regardless as to which Ruby cache coherence protocol is compiled.
- The `m5` Python module now includes functions to set exit events are particular simululation ticks:
    - *setMaxTick(tick)* : Used to to specify the maximum simulation tick.
    - *getMaxTick()* : Used to obtain the maximum simulation tick value.
    - *getTicksUntilMax()*: Used to get the number of ticks remaining until the maximum tick is reached.
    - *scheduleTickExitFromCurrent(tick)* : Used to schedule an exit exit event a specified number of ticks in the future.
    - *scheduleTickExitAbsolute(tick)* : Used to schedule an exit event as a specified tick.
- We now include the `RiscvMatched` board as part of the gem5 stdlib.
This board is modeled after the [HiFive Unmatched board](https://www.sifive.com/boards/hifive-unmatched) and may be used to emulate its behavior.
See "configs/example/gem5_library/riscv-matched-fs.py" and "configs/example/gem5_library/riscv-matched-hello.py" for examples using this board.
- An API for [SimPoints](https://doi.org/10.1145/885651.781076) has been added.
SimPoints can substantially improve gem5 Simulation time by only simulating representative parts of a simulation then extrapolating statistical data accordingly.
Examples of using SimPoints with gem5 can be found in "configs/example/gem5_library/checkpoints/simpoints-se-checkpoint.py" and "configs/example/gem5_library/checkpoints/simpoints-se-restore.py".
- "Workloads" have been introduced to gem5.
Workloads have been incorporated into the gem5 Standard library.
They can be used specify the software to be run on a simulated system that come complete with input parameters and any other dependencies necessary to run a simuation on the target hardware.
At the level of the gem5 configuration script a user may specify a workload via a board's `set_workload` function.
For example, `set_workload(Workload("x86-ubuntu-18.04-boot"))` sets the board to use the "x86-ubuntu-18.04-boot" workload.
This workload specifies a boot consisting of the Linux 5.4.49 kernel then booting an Ubunutu 18.04 disk image, to exit upon booting.
Workloads are agnostic to underlying gem5 design and, via the gem5-resources infrastructure, will automatically retrieve all necessary kernels, disk-images, etc., necessary to execute.
Examples of using gem5 Workloads can be found in "configs/example/gem5_library/x86-ubuntu-ruby.py" and "configs/example/gem5_library/riscv-ubuntu-run.py".
- To aid gem5 developers, we have incorporated [pre-commit](https://pre-commit.com) checks into gem5.
These checks automatically enforce the gem5 style guide on Python files and a subset of other requirements (such as line length) on altered code prior to a `git commit`.
Users may install pre-commit by running `./util/pre-commit-install.sh`.
Passing these checks is a requirement to submit code to gem5 so installation is strongly advised.
- A multiprocessing module has been added.
This allows for multiple simulations to be run from a single gem5 execution via a single gem5 configuration script.
Example of usage found [in this commit message](https://gem5-review.googlesource.com/c/public/gem5/+/63432).
**Note: This feature is still in development.
While functional, it'll be subject to subtantial changes in future releases of gem5**.
- The stdlib's `ArmBoard` now supports Ruby caches.
- Due to numerious fixes and improvements, Ubuntu 22.04 can be booted as a gem5 workload, both in FS and SE mode.
- Substantial improvements have been made to gem5's GDB capabilities.
- The `HBM2Stack` has been added to the gem5 stdlib as a memory component.
- The `MinorCPU` has been fully incorporated into the gem5 Standard Library.
- We now allow for full-system simulation of GPU applications.
The introduction of GPU FS mode allows for the same use-cases as SE mode but reduces the requirement of specific host environments or usage of a Docker container.
The GPU FS mode also has improved simulated speed by functionally simulating memory copies, and provides an easier update path for gem5 developers.
An X86 host and KVM are required to run GPU FS mode.

## API (user facing) changes

- The default CPU Vendor String has been updated to `HygonGenuine`.
This is due to newer versions of GLIBC being more strict about checking current system's supported features.
The previous value, `M5 Simulator`, is not recognized as a valid vendor string and therefore GLIBC returns an error.
- [The stdlib's `_connect_things` funciton call has been moved from the `AbstractBoard`'s constructor to be run as board pre-instantiation process](https://gem5-review.googlesource.com/c/public/gem5/+/65051).
This is to overcome instances where stdlib components (memory, processor, and cache hierarhcy) require Board information known only after its construction.
**This change breaks cases where a user utilizes the stdlib `AbstractBoard` but does not use the stdlib `Simulator` module. This can be fixed by adding the `_pre_instantiate` function before `m5.instantiate`**.
An exception has been added which explains this fix, if this error occurs.
- The setting of checkpoints has been moved from the stdlib's "set_workload" functions to the `Simulator` module.
Setting of checkpoints via the stdlib's "set_workload" functions is now deprecated and will be removed in future releases of gem5.
- The gem5 namespace `Trace` has been renamed `trace` to conform to the gem5 style guide.
- Due to the allowing of multiple ISAs per gem5 build, the `TARGET_ISA`  variable has been replaced with `USE_$(ISA)` variables.
For example, if a build contains both the X86 and ARM ISAs the `USE_X86` and `USE_ARM` variables will be set.

## Big Fixes

- Several compounding bugs were causing bugs with floating point operations within gem5 simulations.
These have been fixed.
- Certain emulated syscalls were behaving incorrectly when using RISC-V due to incorrect `open(2)` flag values.
These values have been fixed.
- The GIVv3 List register mapping has been fixed.
- Access permissions for GICv3 cpu registers have been fixed.
- In previous releases of gem5 the `sim_quantum` value was set for all cores when using the Standard Library.
This caused issues when setting exit events at a particular tick as it resulted in the exit being off by `sim_quantum`.
As such, the `sim_quantum` value is only when using KVM cores.
- PCI ranges in `VExpress_GEM5_Foundation` fixed.
- The `SwitchableProcessor` processor has been fixed to allow switching to a KVM core.
Previously the `SwitchableProcessor` only allowed a user to switch from a KVM core to a non-KVM core.
- The Standard Library has been fixed to permit multicore simulations in SE mode.
- [A bug was fixed in the rcr X86 instruction](https://gem5.atlassian.net/browse/GEM5-1265).

## Build related changes

- gem5 can now be compiled with Scons 4 build system.
- gem5 can now be compiled with Clang version 14 (minimum Clang version 6).
- gem5 can now be compiled with GCC Version 12 (minimum GCC version 7).


## Other minor updates

- The gem5 stdlib examples in "configs/example/gem5_library" have been updated to, where appropriate, use the stdlib's Simulator module.
These example configurations can be used for reference as to how `Simulator` module may be utilized in gem5.
- Granulated SGPR computation has been added for gfx9 gpu-compute.
- The stdlib statistics have been improved:
    - A `get_simstats` function has been added to access statistics from the `Simulator` module.
    - Statistics can be printed: `print(simstats.board.core.some_integer)`.
- GDB ports are now specified for each workload, as opposed to per-simulation run.
- The `m5` utility has been expanded to include "workbegin" and "workend" annotations.
This can be added with `m5 workbegin` and `m5 workend`.
- A `PrivateL1SharedL2CacheHierarchy` has been added to the Standard Library.
- A `GEM5_USE_PROXY` environment variable has been added.
This allows users to specify a socks5 proxy server to use when obtaining gem5 resources and the resources.json file.
It uses the format `<host>:<port>`.
- The fastmodel support has been improved to function with Linux Kernel 5.x.
- The `set_se_binary_workload` function now allows for the passing of input parameters to a binary workload.
- A functional CHI cache hierarchy has been added to the gem5 Standard Library: "src/python/gem5/components/cachehierarchies/chi/private_l1_cache_hierarchy.py".
- The RISC-V K extension has been added.
It includes the following instructions:
  - Zbkx: xperm8, xperm4
  - Zknd: aes64ds, aes64dsm, aes64im, aes64ks1i, aes64ks2
  - Zkne: aes64es, aes64esm, aes64ks1i, aes64ks2
  - Zknh: sha256sig0, sha256sig1, sha256sum0, sha256sum1, sha512sig0, sha512sig1, sha512sum0, sha512sum1
  - Zksed: sm4ed, sm4ks
  - Zksh: sm3p0, sm3p1

# Version 22.0.0.2

**[HOTFIX]** This hotfix contains a set of critical fixes to be applied to gem5 v22.0.
This hotfix:

- Fixes the ARM booting of Linux kernels making use of FEAT_PAuth.
- Removes incorrect `requires` functions in AbstractProcessor and AbstractGeneratorCore.
These `requires` were causing errors when running generators with any ISA other than NULL.
- Fixes the standard library's `set_se_binary_workload` function to exit on Exit Events (work items) by default.
- Connects a previously unconnected PCI port in the example SST RISC-V config to the membus.
- Updates the SST-gem5 README with the correct download links.
- Adds a `getAddrRanges` function to the `HBMCtrl`.
This ensures the XBar connected to the controller can see the address ranges covered by both pseudo channels.
- Fixes test_download_resources.py so the correct parameter is passed to the download test script.

# Version 22.0.0.1

**[HOTFIX]** Fixes relative import in "src/python/gem5/components/processors/simple_core.py".

The import `from python.gem5.utils.requires import requires` in v22.0.0.0 of gem5 is incorrect as it causes problems when executing gem5 binaries  in certain directories (`python` isn't necessary included).
To resolve this, this import has been changed to `from ...utils.requires imports requires`.
This should work in all supported use-cases.

# Version 22.0.0.0

gem5 version 22.0 has been slightly delayed, but we a have a very strong release!
This release has 660 changes from 48 unique contributors.
While there are not too many big ticket features, the community has done a lot to improve the stablity and add bugfixes to gem5 over this release.
That said, we have a few cool new features like full system GPU support, a huge number of Arm improvements, and an improved HBM model.

See below for more details!

## New features

- [Arm now models DVM messages for TLBIs and DSBs accurately](https://gem5.atlassian.net/browse/GEM5-1097). This is implemented in the CHI protocol.
- EL2/EL3 support on by default in ArmSystem
- HBM controller which supports pseudo channels
- [Improved Ruby's SimpleNetwork routing](https://gem5.atlassian.net/browse/GEM5-920)
- Added x86 bare metal workload and better real mode support
- [Added round-robin arbitration when using multiple prefetchers](https://gem5.atlassian.net/browse/GEM5-1169)
- [KVM Emulation added for ARM GIGv3](https://gem5.atlassian.net/browse/GEM5-1138)
- Many improvements to the CHI protocol

## Many RISC-V instructions added

The following RISCV instructions have been added to gem5's RISC-V ISA:

* Zba instructions: add.uw, sh1add, sh1add.uw, sh2add, sh2add.uw, sh3add, sh3add.uw, slli.uw
* Zbb instructions: andn, orn, xnor, clz, clzw, ctz, ctzw, cpop, cpopw, max, maxu, min, minu, sext.b, sext.h, zext.h, rol, rolw, ror, rori, roriw, rorw, orc.b, rev8
* Zbc instructions: clmul, clmulh, clmulr
* Zbs instructions: bclr, bclri, bext, bexti, binv, binvi, bset, bseti
* Zfh instructions: flh, fsh, fmadd.h, fmsub.h, fnmsub.h, fnmadd.h, fadd.h, fsub.h, fmul.h, fdiv.h, fsqrt.h, fsgnj.h, fsgnjn.h, fsgnjx.h, fmin.h, fmax.h, fcvt.s.h, fcvt.h.s, fcvt.d.h, fcvt.h.d, fcvt.w.h, fcvt.h.w, fcvt.wu.h, fcvt.h.wu

### Improvements to the stdlib automatic resource downloader

The gem5 standard library's downloader has been re-engineered to more efficiently obtain the `resources.json` file.
It is now cached instead of retrieved on each resource retrieval.

The `resources.json` directory has been moved to a more permament URL at <http://resources.gem5.org/resources.json>.

Tests have also been added to ensure the resources module continues to function correctly.

### gem5 in SystemC support revamped

The gem5 in SystemC has been revamped to accomodate new research needs.
These changes include stability improvements and bugs fixes.
The gem5 testing suite has also been expanded to include gem5 in SystemC tests.

### Improved GPU support

Users may now simulate an AMD GPU device in full system mode using the ROCm 4.2 compute stack.
Until v21.2, gem5 only supported GPU simulation in Syscall-Emulation mode with ROCm 4.0.
See [`src/gpu-fs/README.md`](https://gem5.googlesource.com/public/gem5-resources/+/refs/heads/stable/src/gpu-fs/) in gem5-resources and example scripts in [`configs/example/gpufs/`](https://gem5.googlesource.com/public/gem5/+/refs/tags/v22.0.0.0/configs/example/gpufs/) for example scripts which run GPU full system simulations.

A [GPU Ruby random tester has been added](https://gem5-review.googlesource.com/c/public/gem5/+/59272) to help validate the correctness of the CPU and GPU Ruby coherence protocols as part of every kokoro check-in.
This helps validate the correctness of the protocols before new changes are checked in.
Currently the tester focuses on the protocols used with the GPU, but the ideas are extensible to other protocols.
The work is based on "Autonomous Data-Race-Free GPU Testing", IISWC 2019, Tuan Ta, Xianwei Zhang, Anthony Gutierrez, and Bradford M. Beckmann.

### An Arm board has been added to the gem5 Standard Library

Via [this change](https://gem5-review.googlesource.com/c/public/gem5/+/58910), an ARM Board, `ArmBoard`, has been added to the gem5 standard library.
This allows for an ARM system to be run using the gem5 stdlib components.

An example gem5 configuration script using this board can be found in `configs/example/gem5_library/arm-ubuntu-boot-exit.py`.

### `createAddrRanges` now supports NUMA configurations

When the system is configured for NUMA, it has multiple memory ranges, and each memory range is mapped to a corresponding NUMA node. For this, the change enables `createAddrRanges` to map address ranges to only a given HNFs.

Jira ticker here: https://gem5.atlassian.net/browse/GEM5-1187.

## API (user-facing) changes

### CPU model types are no longer simply the model name, but they are specialized for each ISA

For instance, the `O3CPU` is now the `X86O3CPU` and `ArmO3CPU`, etc.
This requires a number of changes if you have your own CPU models.
See https://gem5-review.googlesource.com/c/public/gem5/+/52490 for details.

Additionally, this requires changes in any configuration script which inherits from the old CPU types.

In many cases, if there is only a single ISA compiled the old name will still work.
However, this is not 100% true.

Finally, `CPU_MODELS` is no longer a parameter in `build_opts/`.
Now, if you want to compile a CPU model for a particular ISA you will have to add a new file for the CPU model in the `arch/` directory.

### Many changes in the CPU and ISA APIs

If you have any specialized CPU models or any ISAs which are not in the mainline, expect many changes when rebasing on this release.

- No longer use read/setIntReg (e.g., see https://gem5-review.googlesource.com/c/public/gem5/+/49766)
- InvalidRegClass has changed (e.g., see https://gem5-review.googlesource.com/c/public/gem5/+/49745)
- All of the register classes have changed (e.g., see https://gem5-review.googlesource.com/c/public/gem5/+/49764/)
- `initiateSpecialMemCmd` renamed to `initiateMemMgmtCmd` to generalize to other command beyond HTM (e.g., DVM/TLBI)
- `OperandDesc` class added (e.g., see https://gem5-review.googlesource.com/c/public/gem5/+/49731)
- Many cases of `TheISA` have been removed

## Bug Fixes

- [Fixed RISC-V call/ret instruction decoding](https://gem5-review.googlesource.com/c/public/gem5/+/58209). The fix adds IsReturn` and `IsCall` flags for RISC-V jump instructions by defining a new `JumpConstructor` in "standard.isa". Jira Ticket here: https://gem5.atlassian.net/browse/GEM5-1139.
- [Fixed x86 Read-Modify-Write behavior in multiple timing cores with classic caches](https://gem5-review.googlesource.com/c/public/gem5/+/55744). Jira Ticket here: https://gem5.atlassian.net/browse/GEM5-1105.
- [The circular buffer for the O3 LSQ has been fixed](https://gem5-review.googlesource.com/c/public/gem5/+/58649). This issue affected running the O3 CPU with large workloaders. Jira Ticket here: https://gem5.atlassian.net/browse/GEM5-1203.
- [Removed "memory-leak"-like error in RISC-V lr/sc implementation](https://gem5-review.googlesource.com/c/public/gem5/+/55663). Jira issue here: https://gem5.atlassian.net/browse/GEM5-1170.
- [Resolved issues with Ruby's memtest](https://gem5-review.googlesource.com/c/public/gem5/+/56811). In gem5 v21.2, If the size of the address range was smaller than the maximum number of outstandnig requests allowed downstream, the tester would get stuck trying to find a unique address. This has been resolved.

## Build-related changes

- Variable in `env` in the SConscript files now requires you to use `env['CONF']` to access them. Anywhere that `env['<VARIABLE>']` appeared should noe be `env['CONF']['<VARIABLE>']`
- Internal build files are now in a per-target `gem5.build` directory
- All build variable are per-target and there are no longer any shared variables.

## Other changes

- New bootloader is required for Arm VExpress_GEM5_Foundation platform. See https://gem5.atlassian.net/browse/GEM5-1222 for details.
- The MemCtrl interface has been updated to use more inheritance to make extending it to other memory types (e.g., HBM pseudo channels) easier.

# Version 21.2.1.1

**[HOTFIX]** In order to ensure v21 of gem5 remains compatible with future changes, the gem5 stdlib downloader has been updated to obtain the resources.json file from <https://resources.gem5.org/resources.json>.
As this domain is under the gem5 project control, unlike the previous googlesource URL, we can ensure longer-term stability.
The fix also ensures the downloader can parse plain-text JSON and base64 encoding of the resources.json file.

# Version 21.2.1.0

Version 21.2.1 is a minor gem5 release consisting of bug fixes. The 21.2.1 release:

* Fixes a bug in which [a RCV instruction is wrongly regarded as a branch](https://gem5.atlassian.net/browse/GEM5-1137).
* Removes outdated and incomplete standard library documentation.
Users wishing to learn more about the gem5 standard library should consult materials [on the gem5 website](https://www.gem5.org/documentation/gem5-stdlib/overview).
* Adds a VirtIO entropy device (VirtIORng) to RISC-V.
Without this, [RISCV Disk images can take considerable time to boot and occasionally do so in error](https://gem5.atlassian.net/browse/GEM5-1151).
* Removes the 'typing.final' decorator from the standard library.
'typing.final' was introduced in Python 3.8, but v21.2 of gem5 supports Python 3.6.
* Fixes the broken NPB stdlib example test.

# Version 21.2.0.0

## API (user-facing) changes

All `SimObject` declarations in SConscript files now require a `sim_objects` parameter which should list all SimObject classes declared in that file which need c++ wrappers.
Those are the SimObject classes which have a `type` attribute defined.

Also, there is now an optional `enums` parameter which needs to list all of the Enum types defined in that SimObject file.
This should technically only include Enum types which generate c++ wrapper files, but currently all Enums do that so all Enums should be listed.

## Initial release of the "gem5 standard library"

Previous release had an alpha release of the "components library."
This has now been wrapped in a larger "standard library".

The *gem5 standard library* is a Python package which contains the following:

- **Components:** A set of Python classes which wrap gem5's models. Some of the components are preconfigured to match real hardware (e.g., `SingleChannelDDR3_1600`) and others are parameterized. Components can be combined together onto *boards* which can be simulated.
- **Resources:** A set of utilities to interact with the gem5-resources repository/website. Using this module allows you to *automatically* download and use many of gem5's prebuilt resources (e.g., kernels, disk images, etc.).
- **Simulate:** *THIS MODULE IS IN BETA!* A simpler interface to gem5's simulation/run capabilities. Expect API changes to this module in future releases. Feedback is appreciated.
- **Prebuilt**: These are fully functioning prebuilt systems. These systems are built from the components in `components`. This release has a "demo" board to show an example of how to use the prebuilt systems.

Examples of using the gem5 standard library can be found in `configs/example/gem5_library/`.
The source code is found under `src/python/gem5`.

## Many Arm improvements

- [Improved configurability for Arm architectural extensions](https://gem5.atlassian.net/browse/GEM5-1132): we have improved how to enable/disable architectural extensions for an Arm system. Rather than working with indipendent boolean values, we now use a unified ArmRelease object modelling the architectural features supported by a FS/SE Arm simulation
- [Arm TLB can store partial entries](https://gem5.atlassian.net/browse/GEM5-1108): It is now possible to configure an ArmTLB as a walk cache: storing intermediate PAs obtained during a translation table walk.
- [Implemented a multilevel TLB hierarchy](https://gem5.atlassian.net/browse/GEM5-790): enabling users to compose/model a customizable multilevel TLB hierarchy in gem5. The default Arm MMU has now an Instruction L1 TLB, a Data L1 TLB and a Unified (Instruction + Data) L2 TLB.
- Provided an Arm example script for the gem5-SST integration (<https://gem5.atlassian.net/browse/GEM5-1121>).

## GPU improvements

- Vega support: gfx900 (Vega) discrete GPUs are now both supported and tested with [gem5-resources applications](https://gem5.googlesource.com/public/gem5-resources/+/refs/heads/stable/src/gpu/).
- Improvements to the VIPER coherence protocol to fix bugs and improve performance: this improves scalability for large applications running on relatively small GPU configurations, which caused deadlocks in VIPER's L2.  Instead of continually replaying these requests, the updated protocol instead wakes up the pending requests once the prior request to this cache line has completed.
- Additional GPU applications: The [Pannotia graph analytics benchmark suite](https://github.com/pannotia/pannotia) has been added to gem5-resources, including Makefiles, READMEs, and sample commands on how to run each application in gem5.
- Regression Testing: Several GPU applications are now tested as part of the nightly and weekly regressions, which improves test coverage and avoids introducing inadvertent bugs.
- Minor updates to the architecture model: We also added several small changes/fixes to the HSA queue size (to allow larger GPU applications with many kernels to run), the TLB (to create GCN3- and Vega-specific TLBs), adding new instructions that were previously unimplemented in GCN3 and Vega, and fixing corner cases for some instructions that were leading to incorrect behavior.

## gem5-SST bridges revived

We now support gem5 cores connected to SST memory system for gem5 full system mode.
This has been tested for RISC-V and Arm.
See `ext/sst/README.md` for details.

## LupIO devices

LupIO devices were developed by Prof. Joel Porquet-Lupine as a set of open-source I/O devices to be used for teaching.
They were designed to model a complete set of I/O devices that are neither too complex to teach in a classroom setting, or too simple to translate to understanding real-world devices.
Our collection consists of a real-time clock, random number generator, terminal device, block device, system controller, timer device, programmable interrupt controller, as well as an inter-processor interrupt controller.
A more detailed outline of LupIO can be found here: <https://luplab.cs.ucdavis.edu/assets/lupio/wcae21-porquet-lupio-paper.pdf>.
Within gem5, these devices offer the capability to run simulations with a complete set of I/O devices that are both easy to understand and manipulate.

The initial implementation of the LupIO devices are for the RISC-V ISA.
However, they should be simple to extend to other ISAs through small source changes and updating the SConscripts.

## Other improvements

- Removed master/slave terminology: this was a closed ticket which was marked as done even though there were multiple references of master/slave in the config scripts which we fixed.
- Armv8.2-A FEAT_UAO implementation.
- Implemented 'at' variants of file syscall in SE mode (<https://gem5.atlassian.net/browse/GEM5-1098>).
- Improved modularity in SConscripts.
- Arm atomic support in the CHI protocol
- Many testing improvements.
- New "tester" CPU which mimics GUPS.

# Version 21.1.0.2

**[HOTFIX]** [A commit introduced `std::vector` with `resize()` to initialize all storages](https://gem5-review.googlesource.com/c/public/gem5/+/27085).
This caused data duplication in statistics and broke the Vector statistics.
This hotfix initializes using loops which fixes the broken statistics.

# Version 21.1.0.1

**[HOTFIX]** [A "'deprecated' attribute directive ignored" warning was being thrown frequently when trying to build v21.1.0.0](https://gem5.atlassian.net/browse/GEM5-1063). While this issue did not break the build, it made reading the build output difficult and caused confused. As such a patch has been applied to fix this issue.

# Version 21.1.0.0

Since v21.0 we have received 780 commits with 48 unique contributors, closing 64 issues on our [Jira Issue Tracker](https://gem5.atlassian.net/).
In addition to our [first gem5 minor release](#version-21.0.1.0), we have included a range of new features, and API changes which we outline below.

## Added the Components Library [Alpha Release]

The purpose of the gem5 components library is to provide gem5 users a standard set of common and useful gem5 components, pre-built, to add to their experiments.
The gem5 components library adopts a modular architecture design so components may be easily added, removed, and extended, as needed.

Examples of using the gem5 components library can be found in [`configs/example/components-library`](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.1.0.0/configs/example/components-library).

**Important Disclaimer:**
This is a pre-alpha release.
The purpose of this release is to get community feedback.
Though some testing has been done, we expect regular fixes and improvements until the library reaches a stable state.
A Jira Ticket outlining TODOs and known bugs can be found at <https://gem5.atlassian.net/browse/GEM5-648>.

## Improvements to GPU simulation

### ROCm 4.0 support

ROCm 4.0 is now officially supported.

### gfx801 (Carrizo) and gfx803 (Fiji) support

gfx801 (Carrizo) and gfx803 (Fiji) are both supported and tested with the gem5-resources applications.

### Better scoreboarding support

Better scoreboarding support has been added.
This reduces stalls by up to 42%.

## Accuracy and coverage stat added to prefetcher caches

Accuracy and coverage stats have been added for prefetcher caches.
Accuracy is defined as the ratio of the number of prefetch requests counted as useful over the total number of prefetch requests issued.
Coverage is defined as the ratio of the number of prefetch requests counted as useful over the number of useful prefetch request plus the remaining demand misses.

## POWER 64-bit SE mode

The POWER 64-bit ISA is now supported in Syscall Execution mode.

## RISC-V PMP now supported

gem5 now supports simulation of RISC-V Physical Memory Protection (PMP).
Simulations can boot and run Keystone and Eyrie.

## Improvements to the replacement policies

The gem5 replacement policies framework now supports more complex algorithms.
It now allows using addresses, PC, and other information within a policy.

**Note:**
Assuming this information is promptly available at the cache may be unrealistic.

### Set Dueling

Classes that handle set dueling have been created ([Dueler and DuelingMonitor](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.1.0.0/src/mem/cache/tags/dueling.hh)).
They can be used in conjunction with different cache policies.
A [replacement policy that uses it](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.1.0.0/src/mem/cache/replacement_policies/dueling_rp.hh) has been added for guidance.

## RISC-V is now supported as a host machine.

gem5 is now compilable and runnable on a RISC-V host system.

## New Deprecation MARCOs added

Deprecation MACROS have been added for deprecating namespaces (`GEM5_DEPRECATED_NAMESPACE`), and deprecating other MACROs (`GEM5_DEPRECATED_MACRO`).

**Note:**
For technical reasons, using old macros won't produce any deprecation warnings.
## Refactoring of the gem5 Namespaces

Snake case has been adopted as the new convention for name spaces.
As a consequence, multiple namespaces have been renamed:

* `Minor` -> `minor`
* `Loader` -> `loader`
* `Stats` -> `statistics`
* `Enums` -> `enums`
* `Net` -> `networking`
* `ProbePoints` -> `probing`
* `ContextSwitchTaskId` -> `context_switch_task_id`
* `Prefetcher` -> `prefetch`
* `Encoder` -> `encoder`
* `Compressor` -> `compression`
* `QoS` -> `qos`
* `ReplacementPolicy` -> `replacement_policy`
* `Mouse` -> `mouse`
* `Keyboard` -> `keyboard`
* `Int` -> `as_int`
* `Float` -> `as_float`
* `FastModel` -> `fastmodel`
* `GuestABI` -> `guest_abi`
* `LockedMem` -> `locked_mem`
* `DeliveryMode` -> `delivery_mode`
* `PseudoInst` -> `pseudo_inst`
* `DecodeCache` -> `decode_cache`
* `BitfieldBackend` -> `bitfield_backend`
* `FreeBSD` -> `free_bsd`
* `Linux` -> `linux`
* `Units` -> `units`
* `SimClock` -> `sim_clock`
* `BloomFilter` -> `bloom_filter`
* `X86Macroop` -> `x86_macroop`
* `ConditionTests` -> `condition_tests`
* `IntelMP` -> `intelmp`
* `SMBios` -> `smbios`
* `RomLables` -> `rom_labels`
* `SCMI` -> `scmi`
* `iGbReg` -> `igbreg`
* `Ps2` -> `ps2`
* `CopyEngineReg` -> `copy_engine_reg`
* `TxdOp` -> `txd_op`
* `Sinic` -> `sinic`
* `Debug` -> `debug`

In addition some other namespaces were added:

* `gem5::ruby`, for Ruby-related files
* `gem5::ruby::garnet`, for garnet-related files
* `gem5::o3`, for the O3-cpu's related files
* `gem5::memory`, for files related to memories

Finally, the `m5` namespace has been renamed `gem5`.

## MACROs in `base/compiler.hh`

The MACROs in base/compiler.hh of the form `M5_*` have been deprecated and replaced with macros of the form `GEM5_*`, with some other minor name adjustments.

## MemObject Removed

MemObject simobject had been marked for deprecation and has now been officially removed from the gem5 codebase.

## Minimum GCC version increased to 7; minimum Clang version increased to 6; Clang 10 and 11 supported; C++17 supported

GCC version 5 and 6 are no longer supported.
GCC 7 is now the minimum GCC compiler version supported.
This changes allows has allowed us to move to the C++17 standard for development.

In addition, the minimum Clang version has increased to 6, and Clang 10 and 11 are now officially supported.

# Version 21.0.1.0

Version 21.0.1 is a minor gem5 release consisting of bug fixes. The 21.0.1 release:

* Fixes the [GCN-GPU Dockerfile](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/util/dockerfiles/gcn-gpu/Dockerfile) to pull from the v21-0 bucket.
* Fixes the tests to download from the v21-0 bucket instead of the develop bucket.
* Fixes the Temperature class:
    * Fixes [fs_power.py](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/configs/example/arm/fs_power.py), which was producing a ["Temperature is not JSON serializable" error](https://gem5.atlassian.net/browse/GEM5-951).
    * Fixes temperature printing in `config.ini`.
    * Fixes the pybind export for the `from_kelvin` function.
* Eliminates a duplicated name warning in [ClockTick](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/src/systemc/channel/sc_clock.cc).
* Fixes the [Ubuntu 18.04 Dockerfile](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/util/dockerfiles/ubuntu-20.04_all-dependencies/Dockerfile) to use Python3 instead of Python2.
* Makes [verify.py](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/src/systemc/tests/verify.py) compatible with Python3.
* Fixes GCN3_X86 builds for aarch64 hosts.
* Fixes building with `SLICC_HTML=True`.
* Fixes the [cpt_upgrader.py](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/util/cpt_upgrader.py) string formatter.
* Fixes typo in [cpy_upgrader.py](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/util/cpt_upgrader.py) where module `errno` was incorrectly put as `ennro`.
* Sets the `--restore-simpoint-checkpoint` flag default to "False" instead of the ambiguous "None".
* Fixes a nonsensical check in [MOESI_CMP_token-L1cache](https://gem5.googlesource.com/public/gem5/+/refs/tags/v21.0.1.0/src/mem/ruby/protocol/MOESI_CMP_token-L1cache.sm) which caused compilation bugs in Clang-11.
* Fixes the `scx_get_parameter_list` for ARM fast models.
* Fixes [bloated object binaries, known to cause issues during linking](https://gem5.atlassian.net/browse/GEM5-1003).
* Due to LTO causing unacceptably long link times for some users, and stripping debug symbols in some setups, it is no longer enabled by default. It may be enabled with the `--with-lto` flag. The `--no-lto` flag has been removed.

# Version 21.0.0.0

Version 21.0 marks *one full year* of gem5 releases, and on this anniversary, I think we have some of the biggest new features yet!
This has been a very productive release with [100 issues](https://gem5.atlassian.net/), over 813  commits, and 49 unique contributors.

## 21.0 New features

### AMBA CHI protocol implemented in SLICC: Contributed by *Tiago Mück*

This new protocol provides a single cache controller that can be reused at multiple levels of the cache hierarchy and configured to model multiple instances of MESI and MOESI cache coherency protocols.
This implementation is based of Arm’s [AMBA 5 CHI specification](https://static.docs.arm.com/ihi0050/d/IHI0050D_amba_5_chi_architecture_spec.pdf) and provides a scalable framework for the design space exploration of large SoC designs.

See [the gem5 documentation](http://www.gem5.org/documentation/general_docs/ruby/CHI/) for more details.
There is also a [gem5 blog post](http://www.gem5.org/2020/05/29/flexible-cache.html) on this new protocol as well.

### Full support for AMD's GCN3 GPU model

In previous releases, this model was only partially supported.
As of gem5 21.0, this model has been fully integrated and is tested nightly.
This model currently only works in syscall emulation mode and requires using the gcn docker container to get the correct version of the ROCm stack.
More information can be found in [this blog post](http://www.gem5.org/2020/05/27/modern-gpu-applications.html).

With this full support, we are also providing many applications as well.
See [gem5-resources](http://resources.gem5.org/) for more information.

### RISC-V Full system Linux boot support: Contributed by *Peter Yuen*

The RISC-V model in gem5 can now boot unmodified Linux!
Additionally, we have implemented DTB generation and support the Berkeley Boot Loader as the stage 1 boot loader.
We have also released a set of resources for you to get started: <https://gem5.googlesource.com/public/gem5-resources/+/refs/heads/develop/src/riscv-fs/>

### New/Changed APIs

There are multiple places where the developers have reduced boilerplate.

* **[API CHANGE]**: No more `create()` functions! Previously, every `SimObject` required a `<SimObjectParams>::create()` function to be manually defined. Forgetting to do this resulted in confusing errors. Now, this function is created for you automatically. You can still override it if you need to handle any special cases.
* **[API CHANGE]**: `params()`: Rather than defining a typedef and the `params()` function for every `SimObject`, you can now use the `PARAMS` macro.

See <http://doxygen.gem5.org/release/current/classSimObject.html#details> for more details on these two API changes.

* **[API CHANGE]**: All stats are now using *new style* groups instead of the older manual stat interface.
  * The previous API (creating stats that are not part of a `Group`) is still supported, but it is now deprecated.
  * If a stat is not created with the new `Group` API, it may not be automatically dumped using new stat APIs (e.g., the Python API).
  * Next release, there will be a warning for all old-style stats.

### Platforms no longer support

* **[USER-FACING CHANGE]**: Python 2.7 is *no longer supported*. You must use Python 3.6+.
* CLANG minimum version is now 3.9
* Bump minimum C++ to C++14

### Other improvements and new features

* Extra options to build m5ops
* m5term improvements
* There is a new python-based library for handling statistics. This library *works*, but hasn't been thoroughly tested yet. Stay tuned for more on this next release.
* Many improvements and additions to unit tests
* Cleaning up the `StaticInst` type
* Workload API changes
* Many updates and changes to the m5 guest utility
* [Support for running arm64 Linux kernel v5.8](https://gem5.atlassian.net/browse/GEM5-787)
* [Arm SCMI implemented](https://gem5.atlassian.net/browse/GEM5-768)

# Version 20.1.0.5

**[HOTFIX]** This hotfix release fixes three known bugs:

* `src/python/m5/util/convert.py` incorrectly stated kibibytes as 'kiB' instead of 'KiB'. This has been fixed.
* Atomic accesses were not checking the access permission bits in the page table descriptors. They were incorrectly using the nature of the request itself. This is now fixed.
* `num_l2chaches_per_cluster` and `num_cpus_per_cluster` were cast to floats in `configs/ruby/MESI_Three_Level_HTM.py`, which caused errors. This has been fixed so they are correctly cast to integers.

# Version 20.1.0.4

**[HOTFIX]** [gem5 was failing to build with SCons 4.0.1 and 4.1.0](https://gem5.atlassian.net/browse/GEM5-916).
This hotfix makes the necessary changes to `site_scons/site_tools/default.py` for gem5 to compile successfully on these versions of SCons.

# Version 20.1.0.3

**[HOTFIX]** A patch was apply to fix an [error where booting Linux stalled when using the ARM ISA](https://gem5.atlassian.net/browse/GEM5-901).
This fix adds the parameter `have_vhe` to enable FEAT_VHE on demand, and is disabled by default to resolve this issue.

# Version 20.1.0.2

**[HOTFIX]** This hotfix release fixes known two bugs:

* A "ValueError: invalid literal for int() with base..." error was being thrown in certain circumstances due to a non-integer being passed to "MemorySize" via a division operation. This has been rectified.
* An assertion in Stats could be triggered due to a name collision between two ThreadStateStats objects, due to both erroneously sharing the same ThreadID. This has been fixed.

# Version 20.1.0.1

**[HOTFIX]** A patch was applied to fix the Garnet network interface stats.
Previously, the flit source delay was computed using both tick and cycles.
This bug affected the overall behavior of the Garnet Network Model.

# Version 20.1.0.0

Thank you to everyone that made this release possible!
This has been a very productive release with [150 issues](https://gem5.atlassian.net/), over 650  commits (a 25% increase from the 20.0 release), and 58 unique contributors (a 100% increase!).

## Process changes

We are no longer using the "master" branch.
Instead, we will have two branches:

* "stable": This will point to the latest stable release (gem5-20.1 as of today)
* "develop": This is the latest development code that will be merged in to the "stable" branch at each release.

We suggest all *users* use the stable (default) branch.
However, to contribute your fixes and new changes to gem5, it should be contributed to the develop branch.
See CONTRIBUTING.md for more details.

gem5 has also implemented a project code of conduct.
See the CODE-OF-CONDUCT.md file for details.
In the code of conduct "we pledge to act and interact in ways that contribute to an open, welcoming, diverse, inclusive, and healthy community."

## New features in 20.1

### New DRAM interface: Contributed by *Wendy Elsasser*

You can find details about this on the [gem5 blog](http://www.gem5.org/2020/05/27/memory-controller.html) or Wendy's talks on YouTube: [Talk on new interface and NVM](https://www.youtube.com/watch?v=t2PRoZPwwpk) and the [talk on LPDDR5](https://www.youtube.com/watch?v=ttJ9_I_Avyc)

* **[PYTHON API CHANGE]**: The DRAM models are now *DRAM interfaces* which is a child of the *memory controller*. Example change shown [in the blog post](http://www.gem5.org/project/2020/07/18/gem5-20-1.html).
  * The DRAM is split into a memory controller and a DRAM interface
  * `SimpleMemory` is no longer a drop-in replacement for a DRAM-based memory controller.
* LPDDR5 model added
* NVM model added
* New memory controller model that can use both NVM and DRAM

### Improved on-chip interconnect model, HeteroGarnet: Contributed by *Srikant Bharadwaj*

You can find details about this on the [gem5 blog](http://www.gem5.org/2020/05/27/heterogarnet.html) and [Srikant's talk on YouTube](https://www.youtube.com/watch?v=AH9r44r2lHA).

* **[USER-FACING CHANGE]**: The network type options are now "simple" and "garnet" instead of "garnet2.0". (If "garnet2.0" is used, you will get a warning until gem5-20.2)
* Added models for clock domain crossings and serialization/deserialization (SERDES)

### Transactional memory support: Contributed by *Timothy Hayes*

You can find details on the [Jira issue](https://gem5.atlassian.net/browse/GEM5-587)

* gem5 now supports Arm TME (transactional memory extensions)
* Transactional memory is only implemented in the `MESI_Three_Level_HTM` Ruby protocol, and it is only implemented in Ruby.
* This implements a checkpointing mechanism for the architectural state and buffering of speculative memory updates.
* IBM POWER and x86 HTM extensions have *not* been implemented.

### Other new features

* External simulator integrations
  * Added support for DRAMSim3
  * Added back support for DRAMSim2
* Armv8-A Self Hosted Debug extension added
* KVM support for Armv8-A  hosts without GICv2 hardware
* Implemented Secure EL2 for Armv8-A

## Removed features

* Dropped support for mercurial version control

## New supported platforms

* GCC up to 10.2 is now supported. Minimum GCC is now 5.0.
* Clang up to version 9. Minimum Clang is now 3.9.

## Platforms no longer support

* **[USER-FACING CHANGE]**: Python 2 is officially deprecated. We will drop support for Python 2 in the next release. In this release you will get a warning if you're using Python 2.
* **[USER-FACING CHANGE]**: We have dropped support for GCC 4.X
* **[USER-FACING CHANGE]**: We have dropped support for Scons 2.x (Note: this is the default in Ubuntu 16.04. Use pip to install a newer scons.)

See <http://www.gem5.org/documentation/general_docs/building> for gem5's current dependencies.

## Other changes

### Deprecating "master" and "slave"

* **[API CHANGE]**: The names "master" and "slave" have been deprecated
  * Special thanks to Lakin Smith, Shivani Parekh, Eden Avivi, and Emily Brickey.
  * Below is a guide to most of the name changes.
  * The goal was to replace problematic language with more descriptive and precise terms.
* There may be some bugs introduced with this change as there were many places in the Python configurations which relied on "duck typing".
* This change is mostly backwards compatible and warning will be issued until at least gem5-20.2.

```
MasterPort -> RequestorPort
SlavePort -> ResponsePort

xbar.slave -> xbar.cpu_side
xbar.master -> xbar.mem_side

MasterId -> RequestorId
```

### Testing improvements

* We now have Jenkins server (<http://jenkins.gem5.org/>) running nightly and other large tests. Special thanks to Mike Upton for setting this up!
  * Nightly tests run the "long" regression tests (many tests added).
  * Compiler tests run gem5 build for many targets and all compilers once a week.
* Updated CI tester (kokoro) to use a more up-to-date environment.
* Improved the testing infrastructure.
  * Almost all testing resources now available in [gem5-resources repository](https://gem5.googlesource.com/public/gem5-resources/).
  * Generally cleaned up the `tests/` directory in mainline gem5.
  * Many general testlib improvements.

### More changes

* **[PYTHON API CHANGE]**: m5.stats.dump() root argument renamed to roots to reflect the fact that it now takes a list of SimObjects
* **[USER-FACING CHANGE]**: Checkpoint compatibility may be broken by the following changes
  * <https://gem5-review.googlesource.com/c/public/gem5/+/25145>
  * <https://gem5-review.googlesource.com/c/public/gem5/+/31874>
* **[API CHANGE]** Changed `setCPU` to `setThreadContext` in Interrupts
* Added a `Compressor` namespace.
* **[API CHANGE]** The `Callback` class was removed and replaced with C++ lambdas.
* Many objects' stats have been updated to the "new" stats style.
* Many more objects have had their APIs formalized. See <http://www.gem5.org/documentation/general_docs/gem5-apis>

----------------------------------------------------------------------------------------------------

# Version 20.0.0.3

**[HOTFIX]** When using the ARM ISA, gem5 could crash when a guest tried to call m5ops. This was due to `m5ops_base` being incorrectly declared in `src/arch/arm/ArmSystem.py`. A fix was applied to remove this declaration.

# Version 20.0.0.2

**[HOTFIX]** A patch was applied to fix the RubyPrefetcher with MESI_Three_Level. Prior to this fix a segfault occurred.

# Version 20.0.0.1

**[HOTFIX]** A fix was applied to stop incorrect clock frequences being reported due to rounding errors.

# Version 20.0.0.0

Welcome to our first "official" gem5 release!
gem5 v19.0.0.0 was a "test" release, but this one has release notes, so it must be official!

Thank you to everyone that made this release possible!
This has been a very productive release with over [70 issues closed](https://gem5.atlassian.net/), over 500 commits, and 31 unique contributors.
Below are some of the highlights, though I'm sure I've missed some important changes.

## New features

* [gem5-resources repository](https://gem5.googlesource.com/public/gem5-resources/)
  * This new repository will store all of the *sources* (e.g., code) used to create testing and research resources. This includes disk images, testing binaries, kernel binaries, etc.
  * Binaries created with the sources are hosted on dist.gem5.org.
  * Details on the new page for resources: <http://www.gem5.org/documentation/general_docs/gem5_resources>.
* Memory SimObjects can now be initialized using an image file using the image_file parameter.
* **[USER-FACING CHANGE]** The m5 utility has been revamped with a new build system based on scons, tests, and updated and more consistent feature support.
  * To build, now use `scons build/<arch>/out/m5`, not `make`.
  * [Documentation](http://www.gem5.org/documentation/general_docs/m5ops/) coming soon.
* Robust support for marshalling data from a function call inside the simulation to a function within gem5 using a predefined set of rules.
  * Developers can specify an ABI for guest<->simulator calls and then "just call functions".
  * Unifies pseudo-inst, syscall, and other support.
  * Code within gem5 has been updated. However, users which added new pseudo-ops may have to update their code.
* **[PYTHON API CHANGE]** Workload configuration pulled out into its own object, simplifying the System object and making workload configuration more modular and flexible.
  * All full system config/run scripts must be updated (e.g., anything that used the `LinuxX86System` or similar SimObject).
  * Many of the parameters of `System` are now parameters of the `Workload` (see `src/sim/Workload.py`).
    * For instance, many parameters of `LinuxX86System` are now part of `X86FsLinux` which is now the `workload` parameter of the `System` SimObject.
  * See https://gem5-review.googlesource.com/c/public/gem5/+/24283/ and https://gem5-review.googlesource.com/c/public/gem5/+/26466 for more details.
* Sv39 paging has been added to the RISC-V ISA, bringing gem5 close to running Linux on RISC-V.
  * (Some) Baremetal OSes are now supported.
* Improvements to DRAM model:
  * Added support for verifying available command bandwidth.
  * Added support for multi-cycle commands.
  * Added new timing parameters.
  * Added ability to interleave bursts.
  * Added LPDDR5 configurations.
* **[Developer change]** We are beginning to document gem5 APIs.
  * Currently, only SimObjects and the APIs they depend on have been documented.
  * We are using doxygen to mark "stable APIs" and will use manual code review to make sure the APIs stay stable.
  * More information will be coming during gem5-20.1 development.

## Removed features

* Support for the ALPHA ISA has been dropped.
  * All ALPHA ISA code has been removed
  * Old "rcS" scripts for ALPHA have been removed

## New supported platforms

* Compiling and running gem5 with Python 3 is now fully supported.
  * Lots of code changes required for this.
  * There may still be some python code that's not up to date. Please open a [Jira ticket](https://gem5.atlassian.net/) if you find any code that doesn't work with python3.
* gem5 now supports Ubuntu 20.04.
* Compiling gem5 with GCC 8 and 9 is now supported.
* Compiling with clang up to version 9 is now supported.

## Testing improvements

* Scons-based tests have been migrated to the testlib framework.
  * Tests can now be run with `tests/main.py`, except for the unittests.
  * Please consult TESTING.md for more information on how these may be run.
* We are continuing to work on CI tests. Most of the plumbing is there for Google Cloud Build integration. See [the Jira issue](https://gem5.atlassian.net/browse/GEM5-237) for details.

## Other API changes

* **[API CHANGE]** Ruby's prefetcher renamed to RubyPrefetcher.
  * Any SLICC protocols with prefetchers need to be updated.
  * Some config scripts for Ruby protocols with prefetchers may need to be updated.
* **[API CHANGE]** SE mode improvements.
  * Better support for the mmap and related syscalls.
  * A new virtual memory area API for tracking SE mode allocations.
  * When implementing syscalls, the way that guest memory is allocated changes. All code in gem5 is updated, but if there are any external syscalls, they may need be updated.
* **[COMMAND LINE CHANGE]** The `--disk-image` argument to `fs.py` is now optional.
  * However, the disk image names *are no longer implied*.
  * The script still implicitly searches `M5_PATH`, but the name of the disk image must be specified.
* **[API CHANGE]** SLICC `queueMemory` is now `enqueue`.
  * All protocol configs must be updated with another message buffer in the memory controllers (directories).
  * All protocol SLICC files must replace `queueMemoryRead` and `queueMemoryWrite` with `enqueue` to another "special" message buffer named `memQueue`.
  * This allows finite buffering between the cache controllers and DRAMCtrl.
* **[API CHANGE]** Added Prefetcher namespace
  * All prefetchers' names have changed from `*Prefetcher` to `Prefetcher::*`
  * If you have any prefetchers that are not in the gem5 mainline, your code will likely need to be updated.

## Other changes

* Implemented ARMv8.3-CompNum, SIMD complex number extension.
* Support for Arm Trusted Firmware + u-boot with the new VExpress_GEM5_Foundation platform
* Removed author list from source files.
  * This was originally so future people would know who to contact.
  * However, it was difficult to maintain and quickly out of date.
  * Copyright is unchanged.
* Improvements to gem5's power model.
* MESI_Three_Level Ruby protocol bugfixes.
* Ruby functional reads now work in more cases.
* Indirect branch stats work correctly now.
