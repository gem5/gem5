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
 *          Nathan Binkert
 */

#ifndef __ARCH_ALPHA_SYSTEM_HH__
#define __ARCH_ALPHA_SYSTEM_HH__

#include <string>
#include <vector>

#include "base/loader/symtab.hh"
#include "cpu/pc_event.hh"
#include "kern/system_events.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "params/AlphaSystem.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class AlphaSystem : public System
{
  public:
    typedef AlphaSystemParams Params;
    AlphaSystem(Params *p);
    ~AlphaSystem();

  public:

    /**
     * Initialise the state of the system.
     */
    void initState() override;

    /**
     * Serialization stuff
     */
    void serializeSymtab(CheckpointOut &cp) const override;
    void unserializeSymtab(CheckpointIn &cp) override;

    /** Override startup() to provide a path to call setupFuncEvents()
     */
    void startup() override;

    /**
     * Set the m5AlphaAccess pointer in the console
     */
    void setAlphaAccess(Addr access);

    /** console symbol table */
    SymbolTable *consoleSymtab;

    /** pal symbol table */
    SymbolTable *palSymtab;

    /** Object pointer for the console code */
    ObjectFile *console;

    /** Object pointer for the PAL code */
    ObjectFile *pal;

#ifndef NDEBUG
    /** Event to halt the simulator if the console calls panic() */
    BreakPCEvent *consolePanicEvent;
#endif

  protected:
    Tick intrFreq;

    /**
     * Proxy used to copy arguments directly into kernel memory.
     */
    FSTranslatingPortProxy virtProxy;

    const Params *params() const { return (const Params *)_params; }


    /** Setup all the function events. Must be done after init() for Alpha since
     * fixFuncEvent() requires a function port
     */
    virtual void setupFuncEvents();

    /** Add a function-based event to PALcode. */
    template <class T>
    T *
    addPalFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(palSymtab, lbl);
    }

    /** Add a function-based event to the console code. */
    template <class T>
    T *
    addConsoleFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(consoleSymtab, lbl);
    }

    Addr fixFuncEventAddr(Addr addr) override;

  public:
    void setIntrFreq(Tick freq) { intrFreq = freq; }
};

#endif // __ARCH_ALPHA_SYSTEM_HH__

