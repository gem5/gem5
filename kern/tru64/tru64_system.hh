/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __TRU64_SYSTEM_HH__
#define __TRU64_SYSTEM_HH__

#include <map>
#include <vector>

#include "sim/system.hh"
#include "targetarch/isa_traits.hh"

class ExecContext;
class EcoffObject;
class SymbolTable;

class BreakPCEvent;
class BadAddrEvent;
class SkipFuncEvent;
class PrintfEvent;
class DebugPrintfEvent;
class DumpMbufEvent;
class FnEvent;
class AlphaArguments;

class Tru64System : public System
{
  private:
    EcoffObject *kernel;
    EcoffObject *console;

    SymbolTable *kernelSymtab;
    SymbolTable *consoleSymtab;

    BreakPCEvent *kernelPanicEvent;
    BreakPCEvent *consolePanicEvent;
    BadAddrEvent *badaddrEvent;
    SkipFuncEvent *skipPowerStateEvent;
    SkipFuncEvent *skipScavengeBootEvent;
    PrintfEvent *printfEvent;
    DebugPrintfEvent *debugPrintfEvent;
    DebugPrintfEvent *debugPrintfrEvent;
    DumpMbufEvent *dumpMbufEvent;

  private:

    Addr kernelStart;
    Addr kernelEnd;
    Addr kernelEntry;
    bool bin;
    std::vector<string> binned_fns;

  public:
    std::vector<RemoteGDB *>   remoteGDB;
    std::vector<GDBListener *> gdbListen;

  public:
    Tru64System(const std::string _name,
                const uint64_t _init_param,
                MemoryController *_memCtrl,
                PhysicalMemory *_physmem,
                const std::string &kernel_path,
                const std::string &console_path,
                const std::string &palcode,
                const std::string &boot_osflags,
                const bool _bin,
                const std::vector<string> &binned_fns);
    ~Tru64System();

    int registerExecContext(ExecContext *xc);
    void replaceExecContext(ExecContext *xc, int xcIndex);

    Addr getKernelStart() const { return kernelStart; }
    Addr getKernelEnd() const { return kernelEnd; }
    Addr getKernelEntry() const { return kernelEntry; }
    bool breakpoint();

    static void Printf(AlphaArguments args);
    static void DumpMbuf(AlphaArguments args);
};

#endif // __TRU64_SYSTEM_HH__
