/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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
#include <vector>

#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "cpu/pc_event.hh"
#include "kern/system_events.hh"
#include "sim/sim_object.hh"

class MemoryController;
class PhysicalMemory;
class Platform;
class RemoteGDB;
class GDBListener;
class ObjectFile;
class ExecContext;
namespace Kernel { class Binning; }

class System : public SimObject
{
  public:
    MemoryController *memctrl;
    PhysicalMemory *physmem;
    Platform *platform;
    PCEventQueue pcEventQueue;
    uint64_t init_param;

    std::vector<ExecContext *> execContexts;

    /** kernel Symbol table */
    SymbolTable *kernelSymtab;

    /** console symbol table */
    SymbolTable *consoleSymtab;

    /** pal symbol table */
    SymbolTable *palSymtab;

    /** Object pointer for the kernel code */
    ObjectFile *kernel;

    /** Object pointer for the console code */
    ObjectFile *console;

    /** Object pointer for the PAL code */
    ObjectFile *pal;

    /** Begining of kernel code */
    Addr kernelStart;

    /** End of kernel code */
    Addr kernelEnd;

    /** Entry point in the kernel to start at */
    Addr kernelEntry;

    Kernel::Binning *kernelBinning;

#ifdef DEBUG
    /** Event to halt the simulator if the console calls panic() */
    BreakPCEvent *consolePanicEvent;
#endif

  public:
    std::vector<RemoteGDB *> remoteGDB;
    std::vector<GDBListener *> gdbListen;
    bool breakpoint();

  public:
    struct Params
    {
        std::string name;
        MemoryController *memctrl;
        PhysicalMemory *physmem;
        uint64_t init_param;
        bool bin;
        std::vector<std::string> binned_fns;
        bool bin_int;

        std::string kernel_path;
        std::string console_path;
        std::string palcode;
        std::string boot_osflags;

        std::string readfile;
        uint64_t system_type;
        uint64_t system_rev;
    };
    Params *params;

    System(Params *p);
    ~System();

    void startup();

  public:
    /**
     * Returns the addess the kernel starts at.
     * @return address the kernel starts at
     */
    Addr getKernelStart() const { return kernelStart; }

    /**
     * Returns the addess the kernel ends at.
     * @return address the kernel ends at
     */
    Addr getKernelEnd() const { return kernelEnd; }

    /**
     * Returns the addess the entry point to the kernel code.
     * @return entry point of the kernel code
     */
    Addr getKernelEntry() const { return kernelEntry; }

    int registerExecContext(ExecContext *xc);
    void replaceExecContext(ExecContext *xc, int xcIndex);

    void regStats();
    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

  public:
    ////////////////////////////////////////////
    //
    // STATIC GLOBAL SYSTEM LIST
    //
    ////////////////////////////////////////////

    static std::vector<System *> systemList;
    static int numSystemsRunning;

    static void printSystems();
};

#endif // __SYSTEM_HH__
