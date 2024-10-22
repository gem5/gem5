# gem5 Address Translation Remodeling Project

## Goal Statements

In this project we will focus on 2 primary goals:

1. Remodeling the TLB and MMU interfaces to be more generic and consistent
   across ISAs.
2. Add new functionalities such that the address translation pipeline in gem5 is
   up to date with the current state of the art.

## Remodeling Current Implementations

As of today, gem5's implementation of address translation is highly inconsistent
across different ISAs. Each architecture has it's own implementation, with
differing functionalities and separate source files for generic features that
exist regardless of architecture.

Our goal is to move these features to a generic class and leave ISA specific
classes to implement only ISA specific features.

## New Address Translation Functionalities

Currently, gem5's implementation of address translation is highly outdated. gem5
is incapable of simulating multi-level TLBs in both RISCV and x86 architectures,
and does not support translation caches that have been proven to improve address
translation performance. The simulator also does not have support for concurrent
Page Table Walkers and provide very limited data on address translation in
general.

In this project, we plan to implement these new functionalities to gem5, so that
it can be used for address translation research and it's results can be more
similar to current state of the art hardware.
