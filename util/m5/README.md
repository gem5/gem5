The m5 utility provides a command line and library interface for gem5
operations.

These operations are requested by the simulated software through some special
behavior which is recognized by gem5. gem5 will then perform the requested
operation which is outside the normal behavior of the simulated system.



# Trigger mechanisms

There are a few different ways the simulated software can let gem5 know it
wants to perform an operation. Different CPU models have different constraints
depending on how they're implemented, and may not support all of these
different mechanisms.

   Trigger   | Native  | KVM | Fast Model
-------------|---------|-----|------------
 Instruction |   Yes   |     |
 Address     | ARM/X86 | Yes |
 Semihosting |   ARM   |     |   Yes

## "Magic" Instructions

This is the oldest trigger mechanism in gem5, and is supported by all of the
CPU models which interpret instructions one at a time using gem5's ISA
definitions.  It works by co-opting instructions which normally are undefined,
and redefining them to trigger gem5 operations. Exactly what instructions
these are, how they encode what operation they go with, etc., vary from ISA to ISA.

When using the KVM CPU models, the instruction stream is executing on actual
physical hardware which won't treat these instructions specially. They will
retain their old behavior and, most likely, raise an undefined instruction
exception if executed.

Other external models, like ARM's Fast Model CPUs, also won't treat these
instructions specially.

## "Magic" Address Range

This mechanism was added for the KVM CPUs so that they could trigger gem5
operations without having to recognize special instructions. This trigger is
based on a specially set aside range of physical addresses. When a read or
write is targetted at that range, instead of a normal device or memory access,
a gem5 operation is triggered.

Depending on the ISA, gem5 native CPUs should support this mechanism (see the
table below).

When using the KVM CPU, the special range of addresses are not registered as
memory, and so the KVM virtual machine will exit when they're accessed. gem5
will have a chance to recognize the special address, and can trigger the
operation.

When using an external model like ARM's Fast Model CPUs, these external
accesses will leave the CPU complex, and gem5 will be able to recognize them.
Unfortunately if the CPU has multiple threads of execution, gem5 won't be able
to tell which the access came from. Also, the memory access may not happen at a
precise point in the simulated instruction stream due to binary translation.
The architectural state may not be in a consistent state which is suitable to
extract arguments or inject a return value.

### Default address range

Since x86 has a predictable address space layout, the "magic" address range can
be put in a predictable, default location, which is at 0xFFFF0000.

On other architectures, notably ARM, the address space is less predictable, and
it doesn't make sense to set a default location which won't be valid on all
configurations.

## Semihosting

This mechanism was added to support ARM's Fast Model CPUs. It extends ARM's
semihosting support, a mechanism which was already defined to interrupt normal
execution and trigger some sort of behavior in a containing host.

On ISAs which support semihosting (only ARM now, and probably going forward),
gem5 native CPUs can support semihosting instructions, and so should support
the semihosting trigger mechanism.

KVM CPUs use real hardware, and so semihosting instructions will not have
special behavior and will act like their normal counterparts (HLT, etc.).



# Building

## Supported ABIs

To build either the command line utility or one of the versions of the library,
first identify what ABI(s) you're targetting.

   ABI   | Description  | Triggers
---------|--------------|----------
 arm64   | 64 bit ARM   | instruction, adddress, semihosting
 arm     | 32 bit ARM   | instruction
 thumb   | ARM thumb    | instruction
 sparc   | 64 bit SPARC | instruction
 x86     | amd64/x86_64 | instruction, address
 riscv   | 64 bit RISCV | instruction

## SCons

The m5 utility uses a scons based build system. gem5 itself also uses SCons,
but these builds are (mostly) not related and separate.

The SConscript for this utility is set up to use a build directory called
"build", similar to gem5 itself. The build directory is structured so that you
can ask scons to build a portion of it to help narrow down what you want to
build.

### native

There is a **build/native** directory which is for some test binaries which
test generic functionality and are compiled for the host, whatever that happens
to be. These can be run directly, unlike ABI specific tests which may be
possible to run directly depending on the host's architecture, but may not.

### ABI

The first level subdirectories of the build directory (other than "native",
described above) is named after the ABI you're targetting. For instance, build
products for x86 would be in the **build/x86** subdirectory.

Within an ABI subdirectory will be linked copies of all the source files needed
for the build, and also "test" and "out" subdirectories.

#### test

The "test" subdirectory, for instance **build/x86/test**, holds the test
binaries for that ABI in a bin subdirectory, and the results of running those
tests (if requested and possible) in a "result" subdirectory.

#### out

The "out" subdirectory, for instance **build/x86/out**, holds the various final
build products. This includes:

- m5: The command line utility.
- libm5.a: C library.
- gem5OpJni.jar, libgem5OpJni.so, jni/gem5Op.class: Java support files.
- libgem5OpLua.so: Lua module/library.

## Build options

### SCons variables

There are some variables which set build options which need to be controlled on
a per ABI level. Currently, these are:

- CROSS_COMPILE: The cross compiler prefix.
- QEMU_ARCH: The QEMU architecture suffix.

To set these for a particular ABI, prefix the variable name with the ABI's name
and then a dot. For instance, to set the cross compiler prefix to
"x86_64-linux-gnu-" for x86, you would run scons like this:

```shell
scons x86.CROSS_COMPILE=x86_64-linux-gnu- build/x86/out/m5
```

   ABI   | QEMU_ARCH |     CROSS_COMPILE
---------|-----------|---------------------
 arm64   | aarch64   | aarch64-linux-gnu-
 arm     | arm       | arm-linux-gnueabihf-
 thumb   | arm       | arm-linux-gnueabihf-
 sparc   | sparc64   | sparc64-linux-gnu-
 x86     | x86_64    |
 riscv   | riscv64   | riscv64-unknown-linux-gnu-

Note that the default setting for the x86 cross compiler prefix is blank,
meaning that the native/host compiler will be used. If building on a non-x86
host, then you'll need to set an appopriate prefix and may be able to clear
some other prefix corresponding to that host.

### SCons command line flags

--debug-build: Compile with the -g option, and -O0.
--run-tests:   Allow the test result XML files to be build targets.
--verbose:     Show build command lines and full command output.

## External dependency detection

In some cases, if an external dependency isn't detected, the build will
gracefully exclude some targets which depend on it. These include:

### Java support

The SConscript will attempt to find the javac and jar programs. If it can't, it
will disable building the Java support files.

### Lua support

The SConscript will attempt to find lua51 support using pkg-config. If it
can't, it will disable building the lua module/library.

### Non-native tests

The SConscript will attempt to find various QEMU binaries so that it can run
non-native tests using QEMU's application level emulation. The name of the
binary it looks for depends on the ABI and is set to qemu-${QEMU_ARCH}. See
above for a description of per ABI build variables, including QEMU_ARCH.

If it can't find a program with that name, it will disable running non-native
test binaries for that ABI.



# Testing

Tests are based on the googletest system. There are native tests which test
mechanisms which are not specific to any ABI and can be run on the host. These
are built using the native toolchain.

There are also tests for ABI specific mechanisms like the various trigger
types. These will be built using the cross compiler configured for a given ABI.
These tests can be run in QEMU in its application emulation mode, and the build
system can run them automatically if requested and if the required dependencies
have been met.

The tests for the trigger mechanisms can't count on those mechanisms actually
working when running under QEMU, and so will try to set up intercepts which
will catch attempts to use them and verify that they were used correctly. When
running these tests under gem5, set the RUNNING_IN_GEM5 environment variable
which will tell the test to expect the trigger mechanism to actually work.

A junit test exists for the Java jar, in a file named 'OpsTest.java'. That test
can be run on its own through its own main function, or through the junit
framework.



# Command line utility

The command line utility provides a way of triggering gem5 operations either
interactively through a terminal connection to the simulated system, or scripts
running within it.

## Calling syntax

Any call to the utility should have the following structure:

```shell
m5 [call type] <command> [arguments]
```

Call type is optional and selects what trigger mechanism should be used. If
it's omitted, the default mechanism will be used. What the default mechanism is
varies based on the ABI.

   ABI   | Default call type
---------|-------------------
 arm64   | instruction
 arm     | instruction
 thumb   | instruction
 sparc   | instruction
 x86     | address
 riscv   | instruction

The default is usually to use a magic instruction, which for most ABIs is the
only mechanism that's supported, and is what the m5 utility would
tradditionally have used. On x86, the address based mechanism is the default
since it's supported on all current CPU types which also support x86.

### Call type

To override the default call type, you can use one of these arguments.

```shell
--addr [address override]
```

Selects the magic address call type. On most ABIs which don't have a default
magic address range, this argument must be followed by the address range to
use. On x86 if no address is specified, the default (0xFFFF0000) will be used.

```shell
--inst
```

Selects the magic instruction call type.

```shell
--semi
```

Selects the semihosting based call type.

### Commands and arguments

To see a list of commands and the arguments they support, run the utility with
the --help argument.

```shell
m5 --help
```



# C library

The C library provides a set of functions which can trigger gem5 operations
from within compiled programs.

## Building in the library

To use the C library, include the header file located at

```shell
include/gem5/m5ops.h
```

like so:

```shell
#include <gem5/m5ops.h>
```

That will declare the various functions which wrap each of the gem5 operations.
It includes another header file located at

```shell
include/gem5/asm/generic/m5ops.h
```

using a path relative to include. Be sure that include path will resolve based
on the settings of your compiler, or move or modify to fit the existing
options.

As part of the linking step of your application, link in the libm5.a static
library archive which provides the definitions of those functions.

## Trigger mechanisms

The bare function name as defined in the header file will use the magic
instruction based trigger mechanism, what would have historically been the
default.

Some macros at the end of the header file will set up other declarations which
mirror all of the other definitions, but with an "_addr" and "_semi" suffix.
These other versions will trigger the same gem5 operations, but using the
"magic" address or semihosting trigger mechanisms. While those functions will
be unconditionally declared in the header file, a definition will exist in the
library only if that trigger mechanism is supported for that ABI.



# Java jar

In your java source, import the gem5Op class.

```java
import gem5.Ops
```

This class provides a static map named callTypes which map from each of the
call type names ("addr", "inst", or "semi") to an Ops instance. That instance
will provide a set of methods which trigger each of the gem5 operations using
the requested trigger mechanism. The call type "default" maps to whatever the
default call type is for the current ABI.

```shell
gem5.Ops gem5_ops = gem5.Ops.callTypes.get("default");
long sum = gem5_ops.sum(1, 2, 3, 4, 5, 6);
```

To configure the address based trigger mechanism, you can use these static
methods.

void setAddr(long addr);
Set the address for the "magic" address region.

void mapMem();
Map the "magic" physical address region into the process' address space, likely
by mmapping the "/dev/mem" device file.

void unmapMem();
Unmap the "magic" physical address region that was previously mapped.



# lua module

The lua module is implemented in a file called libgem5OpLua.so, and should be
loaded using typical lua mechanisms. It will be built against lua 5.1.

## Integer values

In lua 5.1, all numeric values are (typically) represented as doubles. That
means that 64 bit integer argument values of any type, but in particular
addresses, can't be represented exactly. Calls to gem5 operations using that
type of argument or returning that type of value may not work properly.

In lua 5.3, numeric values can be represented by either a double or a proper
integer without having to rebuild the lua interpreter configured for one or the
other. If the module was ported to lua 5.3 then integer values could be passed
safely.



# Known problems

## Java/lua cross compiling

When building the java or lua modules, a C cross compiler is used so that any
generated binary will be built for the target ABI. Unfortunately, other tools,
headers, etc, come from the host and may not be useable, or worse may be
subtley broken, when used to target a different ABI. To build these objects
correctly, we would need to use a proper cross build environment for their
corresponding languages. Something like this could likely be set up using a
tool like buildroot.
