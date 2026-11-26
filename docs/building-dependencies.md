# Supported build environments

gem5 supports Ubuntu LTS releases that remain in standard support. Ubuntu
24.04 and 26.04 are the supported build environments after Ubuntu 22.04's
standard support ends in May 2027.

The oldest supported LTS sets dependency minimums. Dependencies from the
newest supported LTS must also work. Compiler support is expressed as a range
of major versions; other dependency minimums use major and minor versions.

## Required dependencies

| Dependency | Supported version |
| --- | --- |
| GCC or Clang | GCC 13 through 16, or Clang 18 through 22 |
| Python interpreter and development headers | 3.12+ |
| SCons | 4.5+ |
| zlib | 1.3+ |
| GNU m4 | 1.4+ |
| Git, for obtaining and working with the source | 2.43+ |

## Optional dependencies and development tools

The following minimums follow Ubuntu 24.04's default packages. A dependency
only needs to be installed when using the corresponding optional feature or
tool. Existing feature probes disable unavailable optional features.

| Dependency or tool | Minimum version |
| --- | --- |
| Protocol Buffers, for trace capture and playback | 3.21+ |
| Boost, for SystemC support | 1.83+ |
| HDF5, for statistics output | 1.10+ |
| Capstone, for instruction disassembly | 4.0+ |
| libpng, for PNG framebuffers | 1.6+ |
| elfutils libelf | 0.190+ |
| google-perftools, for tcmalloc | 2.15+ |
| pkg-config | 1.8+ |
| CMake, for DRAMSys | 3.28+ |
| Doxygen | 1.9+ |
| wget | 1.21+ |
| pydot | 1.4+ |
| mypy | 1.9+ |
| pre-commit | 3.6+ |
| clang-format | 18+ |

The package versions are available from the
[Ubuntu 24.04 package index](https://packages.ubuntu.com/noble/).
Ubuntu's [release cycle](https://ubuntu.com/about/release-cycle) describes the
standard support periods used by this policy.

## Containers and older releases

The maintained dependency images are based on Ubuntu 24.04 and 26.04. The
Ubuntu 22.04 image and compiler targets below the supported ranges are no
longer built from this repository. Published image tags for older gem5
releases remain available; use the tag matching the release being reproduced.

This policy covers the environment used to build and run the simulator.
Simulated guest operating systems and the runner VM infrastructure have
separate compatibility requirements.
