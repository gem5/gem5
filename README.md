# Fetch Directed Instruction Prefetching for gem5

This is the development repository for **Fetch Directed Instruction Prefetching
(FDP)** in gem5 also known as **decoupled front-end**. FDP was originally
published in [MICRO'99](https://web.eecs.umich.edu/~taustin/papers/MICRO32-fdp.pdf).
Today, FDP is the standard front-end design for high performance
server CPU's including CPU's from
[Intel](https://www.anandtech.com/show/16881/a-deep-dive-into-intels-alder-lake-microarchitectures/3),
[IBM](https://doi.org/10.1109/ISCA45697.2020.00014),
[AMD](https://www.amd.com/en/support/tech-docs/software-optimization-guide-for-the-amd-zen4-microarchitecture), and [ARM](https://doi.org/10.1109/MM.2020.2972222).

This implementation is based on industrial [report](https://ieeexplore.ieee.org/document/9408197)
and aims to establish a state-of-the-art front-end design comparable to modern servers in gem5.

## How to use
We included an example [script](./configs/example/gem5_library/fdp-hello.py)
how to configure FDP correctly and simulate a simple "Hello World!" program.
The script is tested with the x86, ARM, RISC-V ISA's which can be specified
by the `--isa` flag.

```bash
# Build gem5
scons build/ALL/gem5.opt
# Run the simulation
./build/ALL/gem5.opt \
    configs/example/gem5_library/fdp-hello.py \
        --isa <X86/Arm/RiscV> [--disable-fdp]
```

## Notes
The implementation was develped for the X86 architecture in full-system mode.
Other architectures where not extensively tested. If you encounter any issues,
please let us know via issues, pull requests or PM to David Schall
[GitHub](https://github.com/dhschall),[web page](https://dhschall.github.io/).
Furthermore, we welcome any feedback and ideas for improvements.


## Reference our work
This work was done for our recent [paper](https://ease-lab.github.io/ease_website/pubs/IGNITE_MICRO23.pdf)
accepted to MICRO'23. If you use our work, please cite our paper:
```
@inproceedings{10.1145/3613424.3614258,
  author    = {Schall, David and
               Sandberg, Andreas and
               Grot, Boris},
  title     = {Warming Up a Cold Front-End with Ignite},
  year      = {2023},
  publisher = {Association for Computing Machinery},
  address   = {Toronto, ON, Canada},
  doi       = {10.1145/3613424.3614258},
  booktitle = {Proceedings of the 56th Annual IEEE/ACM International Symposium on Microarchitecture (MICRO '23)},
  series    = {MICRO'23}
}
```

---
---

# The gem5 Simulator

This is the repository for the gem5 simulator. It contains the full source code
for the simulator and all tests and regressions.

The gem5 simulator is a modular platform for computer-system architecture
research, encompassing system-level architecture as well as processor
microarchitecture. It is primarily used to evaluate new hardware designs,
system software changes, and compile-time and run-time system optimizations.

The main website can be found at <http://www.gem5.org>.

## Getting started

A good starting point is <http://www.gem5.org/about>, and for
more information about building the simulator and getting started
please see <http://www.gem5.org/documentation> and
<http://www.gem5.org/documentation/learning_gem5/introduction>.

## Building gem5

To build gem5, you will need the following software: g++ or clang,
Python (gem5 links in the Python interpreter), SCons, zlib, m4, and lastly
protobuf if you want trace capture and playback support. Please see
<http://www.gem5.org/documentation/general_docs/building> for more details
concerning the minimum versions of these tools.

Once you have all dependencies resolved, execute
`scons build/ALL/gem5.opt` to build an optimized version of the gem5 binary
(`gem5.opt`) containing all gem5 ISAs. If you only wish to compile gem5 to
include a single ISA, you can replace `ALL` with the name of the ISA. Valid
options include `ARM`, `NULL`, `MIPS`, `POWER`, `SPARC`, and `X86` The complete
list of options can be found in the build_opts directory.

See https://www.gem5.org/documentation/general_docs/building for more
information on building gem5.

## The Source Tree

The main source tree includes these subdirectories:

* build_opts: pre-made default configurations for gem5
* build_tools: tools used internally by gem5's build process.
* configs: example simulation configuration scripts
* ext: less-common external packages needed to build gem5
* include: include files for use in other programs
* site_scons: modular components of the build system
* src: source code of the gem5 simulator. The C++ source, Python wrappers, and Python standard library are found in this directory.
* system: source for some optional system software for simulated systems
* tests: regression tests
* util: useful utility programs and files

## gem5 Resources

To run full-system simulations, you may need compiled system firmware, kernel
binaries and one or more disk images, depending on gem5's configuration and
what type of workload you're trying to run. Many of these resources can be
obtained from <https://resources.gem5.org>.

More information on gem5 Resources can be found at
<https://www.gem5.org/documentation/general_docs/gem5_resources/>.

## Getting Help, Reporting bugs, and Requesting Features

We provide a variety of channels for users and developers to get help, report
bugs, requests features, or engage in community discussions. Below
are a few of the most common we recommend using.

* **GitHub Discussions**: A GitHub Discussions page. This can be used to start
discussions or ask questions. Available at
<https://github.com/orgs/gem5/discussions>.
* **GitHub Issues**: A GitHub Issues page for reporting bugs or requesting
features. Available at <https://github.com/gem5/gem5/issues>.
* **Jira Issue Tracker**: A Jira Issue Tracker for reporting bugs or requesting
features. Available at <https://gem5.atlassian.net/>.
* **Slack**: A Slack server with a variety of channels for the gem5 community
to engage in a variety of discussions. Please visit
<https://www.gem5.org/join-slack> to join.
* **gem5-users@gem5.org**: A mailing list for users of gem5 to ask questions
or start discussions. To join the mailing list please visit
<https://www.gem5.org/mailing_lists>.
* **gem5-dev@gem5.org**: A mailing list for developers of gem5 to ask questions
or start discussions. To join the mailing list please visit
<https://www.gem5.org/mailing_lists>.

## Contributing to gem5

We hope you enjoy using gem5. When appropriate we advise charing your
contributions to the project. <https://www.gem5.org/contributing> can help you
get started. Additional information can be found in the CONTRIBUTING.md file.
