/*
 * Copyright (c) 2001-2004 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file
 * System Console Interface
 */

#ifndef __ALPHA_CONSOLE_HH__
#define __ALPHA_CONSOLE_HH__

#include "base/range.hh"
#include "dev/alpha_access.h"
#include "dev/io_device.hh"
#include "sim/host.hh"
#include "sim/sim_object.hh"

class BaseCPU;
class SimConsole;
class System;
class SimpleDisk;

/**
 * Memory mapped interface to the system console. This device
 * represents a shared data region between the OS Kernel and the
 * System Console.
 *
 * The system console is a small standalone program that is initially
 * run when the system boots.  It contains the necessary code to
 * access the boot disk, to read/write from the console, and to pass
 * boot parameters to the kernel.
 *
 * This version of the system console is very different from the one
 * that would be found in a real system.  Many of the functions use
 * some sort of backdoor to get their job done.  For example, reading
 * from the boot device on a real system would require a minimal
 * device driver to access the disk controller, but since we have a
 * simulator here, we are able to bypass the disk controller and
 * access the disk image directly.  There are also some things like
 * reading the kernel off the disk image into memory that are normally
 * taken care of by the console that are now taken care of by the
 * simulator.
 *
 * These shortcuts are acceptable since the system console is
 * primarily used doing boot before the kernel has loaded its device
 * drivers.
 */
class AlphaConsole : public PioDevice
{
  protected:
    union {
        AlphaAccess *alphaAccess;
        uint8_t *consoleData;
    };

    /** the disk must be accessed from the console */
    SimpleDisk *disk;

    /** the system console (the terminal) is accessable from the console */
    SimConsole *console;

    /** a pointer to the system we are running in */
    System *system;

    /** a pointer to the CPU boot cpu */
    BaseCPU *cpu;

    Addr addr;
    static const Addr size = 0x80; // equal to sizeof(alpha_access);

  public:
    /** Standard Constructor */
    AlphaConsole(const std::string &name, SimConsole *cons, SimpleDisk *d,
                 System *s, BaseCPU *c, Platform *platform,
                 int num_cpus, MemoryController *mmu, Addr addr,
                 HierParams *hier, Bus *bus);

    virtual void init();

    /**
     * memory mapped reads and writes
     */
    virtual Fault read(MemReqPtr &req, uint8_t *data);
    virtual Fault write(MemReqPtr &req, const uint8_t *data);

    /**
     * standard serialization routines for checkpointing
     */
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

  public:
    Tick cacheAccess(MemReqPtr &req);
};

#endif // __ALPHA_CONSOLE_HH__
