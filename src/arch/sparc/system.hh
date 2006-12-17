/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#ifndef __ARCH_SPARC_SYSTEM_HH__
#define __ARCH_SPARC_SYSTEM_HH__

#include <string>
#include <vector>

#include "base/loader/symtab.hh"
#include "cpu/pc_event.hh"
#include "kern/system_events.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class SparcSystem : public System
{
  public:
    struct Params : public System::Params
    {
        PhysicalMemory *rom;
        PhysicalMemory *nvram;
        PhysicalMemory *hypervisor_desc;
        PhysicalMemory *partition_desc;
        Addr reset_addr;
        Addr hypervisor_addr;
        Addr openboot_addr;
        Addr nvram_addr;
        Addr hypervisor_desc_addr;
        Addr partition_desc_addr;
        std::string reset_bin;
        std::string hypervisor_bin;
        std::string openboot_bin;
        std::string nvram_bin;
        std::string hypervisor_desc_bin;
        std::string partition_desc_bin;
        std::string boot_osflags;
    };

    SparcSystem(Params *p);

    ~SparcSystem();

    virtual bool breakpoint();

/**
 * Serialization stuff
 */
  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    /** reset binary symbol table */
    SymbolTable *resetSymtab;

    /** hypervison binary symbol table */
    SymbolTable *hypervisorSymtab;

    /** openboot symbol table */
    SymbolTable *openbootSymtab;

    /** nvram symbol table? */
    SymbolTable *nvramSymtab;

    /** hypervisor desc symbol table? */
    SymbolTable *hypervisorDescSymtab;

    /** partition desc symbol table? */
    SymbolTable *partitionDescSymtab;

    /** Object pointer for the reset binary */
    ObjectFile *reset;

    /** Object pointer for the hypervisor code */
    ObjectFile *hypervisor;

    /** Object pointer for the openboot code */
    ObjectFile *openboot;

    /** Object pointer for the nvram image */
    ObjectFile *nvram;

    /** Object pointer for the hypervisor description image */
    ObjectFile *hypervisor_desc;

    /** Object pointer for the partition description image */
    ObjectFile *partition_desc;

    /** System Tick for syncronized tick across all cpus. */
    Tick sysTick;

    /** functional port to ROM */
    FunctionalPort funcRomPort;

    /** functional port to nvram */
    FunctionalPort funcNvramPort;

    /** functional port to hypervisor description */
    FunctionalPort funcHypDescPort;

    /** functional port to partition description */
    FunctionalPort funcPartDescPort;

  protected:
    const Params *params() const { return (const Params *)_params; }

    /** Add a function-based event to reset binary. */
    template <class T>
    T *addResetFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(resetSymtab, lbl);
    }

    /** Add a function-based event to the hypervisor. */
    template <class T>
    T *addHypervisorFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(hypervisorSymtab, lbl);
    }

    /** Add a function-based event to the openboot. */
    template <class T>
    T *addOpenbootFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(openbootSymtab, lbl);
    }

    virtual Addr fixFuncEventAddr(Addr addr)
    {
        //XXX This may eventually have to do something useful.
        return addr;
    }
};

#endif

