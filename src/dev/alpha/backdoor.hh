/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 */

/** @file
 * System Console Backdoor Interface
 */

#ifndef __DEV_ALPHA_BACKDOOR_HH__
#define __DEV_ALPHA_BACKDOOR_HH__

#include "base/types.hh"
#include "dev/alpha/access.h"
#include "dev/io_device.hh"
#include "params/AlphaBackdoor.hh"
#include "sim/sim_object.hh"

class BaseCPU;
class Terminal;
class AlphaSystem;
class SimpleDisk;

/**
 * Memory mapped interface to the system console. This device
 * represents a shared data region between the OS Kernel and the
 * System Console Backdoor.
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
class AlphaBackdoor : public BasicPioDevice
{
  protected:
    struct Access : public AlphaAccess, public Serializable
    {
        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    union {
        Access *alphaAccess;
        uint8_t *consoleData;
    };

    /** the disk must be accessed from the console */
    SimpleDisk *disk;

    /** the system console (the terminal) is accessable from the console */
    Terminal *terminal;

    /** a pointer to the system we are running in */
    AlphaSystem *system;

    /** a pointer to the CPU boot cpu */
    BaseCPU *cpu;

  public:
    typedef AlphaBackdoorParams Params;
    AlphaBackdoor(const Params *p);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void startup() override;

    /**
     * memory mapped reads and writes
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    /**
     * standard serialization routines for checkpointing
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __DEV_ALPHA_BACKDOOR_HH__
