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
 */

#ifndef __KERN_TRU64_TRU64_SYSTEM_HH__
#define __KERN_TRU64_TRU64_SYSTEM_HH__

#include "arch/alpha/system.hh"
#include "arch/isa_traits.hh"
#include "sim/system.hh"

class ExecContext;

class BreakPCEvent;
class BadAddrEvent;
class SkipFuncEvent;
class PrintfEvent;
class DebugPrintfEvent;
class DumpMbufEvent;
class AlphaArguments;

class Tru64System : public AlphaSystem
{
  private:
#ifdef DEBUG
    /** Event to halt the simulator if the kernel calls panic()  */
    BreakPCEvent *kernelPanicEvent;
#endif

    BadAddrEvent *badaddrEvent;
    SkipFuncEvent *skipPowerStateEvent;
    SkipFuncEvent *skipScavengeBootEvent;
    PrintfEvent *printfEvent;
    DebugPrintfEvent  *debugPrintfEvent;
    DebugPrintfrEvent *debugPrintfrEvent;
    DumpMbufEvent *dumpMbufEvent;

  public:
    Tru64System(Params *p);
    ~Tru64System();

    static void Printf(AlphaArguments args);
    static void DumpMbuf(AlphaArguments args);
};

#endif // __KERN_TRU64_TRU64_SYSTEM_HH__
