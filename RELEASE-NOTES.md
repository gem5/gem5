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
