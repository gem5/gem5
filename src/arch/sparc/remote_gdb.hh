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
 * Authors: Nathan Binkert
 */

#ifndef __ARCH_SPARC_REMOTE_GDB_HH__
#define __ARCH_SPARC_REMOTE_GDB_HH__

#include <map>

#include "arch/sparc/types.hh"
#include "base/pollevent.hh"
#include "base/remote_gdb.hh"
#include "cpu/pc_event.hh"

class System;
class ThreadContext;

namespace SparcISA
{

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    enum RegisterConstants
    {
        RegG0 = 0, RegO0 = 8, RegL0 = 16, RegI0 = 24,
        RegF0 = 32,
        RegPc = 64, RegNpc, RegState, RegFsr, RegFprs, RegY,
        /*RegState contains data in same format as tstate */
        Reg32Y = 64, Reg32Psr = 65, Reg32Tbr = 66, Reg32Pc = 67,
        Reg32Npc = 68, Reg32Fsr = 69, Reg32Csr = 70,
        NumGDBRegs
    };

  public:
    RemoteGDB(System *system, ThreadContext *context);

    bool acc(Addr addr, size_t len);

  protected:
    void getregs();
    void setregs();
};

}

#endif /* __ARCH_SPARC_REMOTE_GDB_H__ */
