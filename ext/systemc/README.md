Overview
========

This repository is a redistribution of the Accellera SystemC 2.3.1 library
[[1]][sysc]. This distribution is integrated into the gem5 build system
(CMake) [[2]][gem5].

The repository contains all the source files from the Accellera distribution,
but strips down the boost dependencies. All references to the boost library
are replaced by calls to the C++11 STL. This repository also contains the
TLM 2.0 protocl checker from Doulos [[3]][doulos].

Build
=====

SystemC is built automatically as part of the gem5 CMake build. To build
gem5 with SystemC support:

```sh
cmake -G Ninja --preset opt-all -B build
ninja -C build
```

[sysc]: http://accellera.org/downloads/standards/systemc
[gem5]: http://www.gem5.org/
[doulos]: https://www.doulos.com/knowhow/systemc/tlm2/base_protocol_checker/
