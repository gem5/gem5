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
