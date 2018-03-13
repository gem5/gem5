/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 *          Jaidev Patwardhan
 *          Robert Scheffel
 */

#ifndef __ARCH_RISCV_SYSTEM_HH__
#define __ARCH_RISCV_SYSTEM_HH__

#include <string>
#include <vector>

#include "base/loader/hex_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/pc_event.hh"
#include "kern/system_events.hh"
#include "params/RiscvSystem.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

class RiscvSystem : public System
{
  protected:
    // checker for bare metal application
    bool _isBareMetal;
    // entry point for simulation
    Addr _resetVect;

  public:
    typedef RiscvSystemParams Params;
    RiscvSystem(Params *p);
    ~RiscvSystem();

    // return reset vector
    Addr resetVect() const { return _resetVect; }

    // return bare metal checker
    bool isBareMetal() const { return _isBareMetal; }

    virtual bool breakpoint();

  public:

    /**
     * Set the m5RiscvAccess pointer in the console
     */
    void setRiscvAccess(Addr access);

    /** console symbol table */
    SymbolTable *consoleSymtab;

    /** Object pointer for the console code */
    ObjectFile *console;

    /** Used by some Bare Iron Configurations */
    HexFile *hexFile;

#ifndef NDEBUG
  /** Event to halt the simulator if the console calls panic() */
    BreakPCEvent *consolePanicEvent;
#endif

  protected:
    const Params *params() const { return (const Params *)_params; }

    /** Add a function-based event to the console code. */
    template <class T>
    T *
    addConsoleFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(consoleSymtab, lbl);
    }

    virtual Addr fixFuncEventAddr(Addr addr);

};

#endif

