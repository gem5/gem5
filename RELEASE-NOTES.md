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
