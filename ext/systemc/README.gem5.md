Overview
========

This subfolder (/ext/systemc)  is a custom redistribution of the Accellera
SystemC 2.3.1 library [[1]][sysc]. This distribution replaces Accellera's
Autoconf build system with a SCons build system, which is used by gem5.

In the past it happened several times that some changes in gem5 broke the
SystemC coupling. Recently Accelera has changed the licence for SystemC from
their own licence to Apache2.0, which is compatible with gem5. However, SystemC
usually relies on the Boost library. The repository contains all the source
files from the Accellera distribution, but strips down the boost dependencies,
shown here:[[3]][strip]. All references to the boost library are replaced by
calls to the C++11 STL. This repository also contains the TLM 2.0
protocol-checker from Doulos [[4]][doulos].


[sysc]: http://accellera.org/downloads/standards/systemc
[gem5]: http://www.gem5.org/
[doulos]: https://www.doulos.com/knowhow/systemc/tlm2/base_protocol_checker/
[strip]: https://github.com/tud-ccc/systemc-scons/commit/913a7451939dc4d4bd752df7081064f9f870517a
