/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_ALPHA_REGFILE_HH__
#define __ARCH_ALPHA_REGFILE_HH__

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/intregfile.hh"
#include "arch/alpha/miscregfile.hh"
#include "arch/alpha/types.hh"
#include "arch/alpha/mt.hh"
#include "sim/faults.hh"

#include <string>

//XXX These should be implemented by someone who knows the alpha stuff better

class Checkpoint;
class EventManager;
class ThreadContext;

namespace AlphaISA {

class RegFile {
  protected:
    Addr pc;   // program counter
    Addr npc;  // next-cycle program counter
    Addr nnpc; // next next-cycle program counter

  public:
    Addr
    readPC()
    {
        return pc;
    }

    void
    setPC(Addr val)
    {
        pc = val;
    }

    Addr
    readNextPC()
    {
        return npc;
    }

    void
    setNextPC(Addr val)
    {
        npc = val;
    }

    Addr
    readNextNPC()
    {
        return npc + sizeof(MachInst);
    }

    void
    setNextNPC(Addr val)
    { }

  public:
#if FULL_SYSTEM
    int intrflag;                   // interrupt flag
#endif // FULL_SYSTEM

    void
    clear()
    {
    }

    void serialize(EventManager *em, std::ostream &os);
    void unserialize(EventManager *em, Checkpoint *cp,
        const std::string &section);
};

void copyRegs(ThreadContext *src, ThreadContext *dest);

void copyMiscRegs(ThreadContext *src, ThreadContext *dest);

} // namespace AlphaISA

#endif // __ARCH_ALPHA_REGFILE_HH__
