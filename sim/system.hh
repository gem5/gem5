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

#ifndef __SYSTEM_HH__
#define __SYSTEM_HH__

#include <string>

#include "sim/sim_object.hh"
#include "cpu/pc_event.hh"
#include "base/loader/symtab.hh"

class MemoryController;
class PhysicalMemory;
class RemoteGDB;
class GDBListener;

class ExecContext;

class System : public SimObject
{
  private:

    SymbolTable *kernelSymtab;
    SymbolTable *consoleSymtab;

    BreakPCEvent kernel_panic_event;
    BreakPCEvent console_panic_event;
    BadAddrEvent badaddr_event;
    SkipFuncEvent skip_power_state;
    SkipFuncEvent skip_scavenge_boot;
    PrintfEvent printf_event;
    DebugPrintfEvent debug_printf_event;
    DebugPrintfEvent debug_printfr_event;
    DumpMbufEvent dump_mbuf_event;

    RegFile *initRegs;

    Addr kernelStart;
    Addr kernelEnd;
    Addr kernelEntry;

  public:

    MemoryController *memCtrl;
    PhysicalMemory *physmem;

    PCEventQueue pcEventQueue;

    ExecContext *xc_array[12/*MAX_CPUS*/];
    int num_cpus;

    RemoteGDB *remoteGDB;
    GDBListener *gdbListen;

    System(const std::string name,
           MemoryController *, PhysicalMemory *,
           const std::string &kernel_path, const std::string &console_path,
           const std::string &palcode, const std::string &boot_osflags);

    ~System();

    const SymbolTable *getKernelSymtab() const { return kernelSymtab; }
    const SymbolTable *getConsoleSymtab() const { return consoleSymtab; }

    Addr getKernelStart() const { return kernelStart; }
    Addr getKernelEnd() const { return kernelEnd; }
    Addr getKernelEntry() const { return kernelEntry; }

    void initBootContext(ExecContext *xc);
    void registerExecContext(ExecContext *xc);

    ////////////////////////////////////////////
    //
    // STATIC GLOBAL SYSTEM LIST
    //
    ////////////////////////////////////////////

  public:

    static std::vector<System *> systemList;

    static int numSystemsRunning;

    static void printSystems();
};

#endif // __SYSTEM_HH__
