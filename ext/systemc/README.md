Overview
========

This repository is a redistribution of the Accellera SystemC 2.3.1 library
[[1]][sysc]. This distribution replaces Accellera's Autoconf build system with
a SCons build system, which is very useful for integration of SystemC in other
SCons based projects, e.g., gem5 [[2]][gem5].

The repository contains all the source files from the Accellera distribution,
but strips down the boost dependencies. All references to the boost library
are replaced by calls to the C++11 STL. This repository also contains the
TLM 2.0 protocl checker from Doulos [[3]][doulos].

Build
=====

To build libsystemc-2.3.1.so, simply type scons. Optionally you can specify the
number of jobs.

```
scons -j N
```

To build and link to SystemC from another SCons project, simply call the
SConscript located in `src/`. Be sure to add `-std=c++11` to the `CXXFLAGS` of
your environment and to export the environment as `'env'`. In case you build on
OS X, you will need to add `-undefined dynamic lookup` to your `LINKFLAGS`.
This is how a minimal SConstruct for your SystemC project could look:

```python
env = Environment()

env.Append(CXXFLAGS=['-std=c++11'])
if env['PLATFORM'] == 'darwin':
    env.Append(LINKFLAGS=['-undefined', 'dynamic_lookup'])

systemc = env.SConscript('<path_to_systemc>/src/SConscript', exports=['env'])
env.Program('example', ['example.cc', systemc])
```

[sysc]: http://accellera.org/downloads/standards/systemc
[gem5]: http://www.gem5.org/Main_Page
[doulos]: https://www.doulos.com/knowhow/systemc/tlm2/base_protocol_checker/
