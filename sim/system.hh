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
 */

#ifndef __SYSTEM_HH__
#define __SYSTEM_HH__

#include <string>
#include <vector>

#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/statistics.hh"
#include "cpu/pc_event.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"
#if FULL_SYSTEM
#include "kern/system_events.hh"
#endif

class BaseCPU;
class ExecContext;
class MemoryController;
class ObjectFile;
class MemObject;

#if FULL_SYSTEM
class Platform;
class GDBListener;
class RemoteGDB;
namespace Kernel { class Binning; }
#endif

class System : public SimObject
{
  public:
    MemObject *physmem;
    PCEventQueue pcEventQueue;

    std::vector<ExecContext *> execContexts;
    int numcpus;

    int getNumCPUs()
    {
        if (numcpus != execContexts.size())
            panic("cpu array not fully populated!");

        return numcpus;
    }

#if FULL_SYSTEM
    MemoryController *memctrl;
    Platform *platform;
    uint64_t init_param;

    /** Port to physical memory used for writing object files into ram at
     * boot.*/
    FunctionalPort functionalPort;

    /** kernel symbol table */
    SymbolTable *kernelSymtab;

    /** Object pointer for the kernel code */
    ObjectFile *kernel;

    /** Begining of kernel code */
    Addr kernelStart;

    /** End of kernel code */
    Addr kernelEnd;

    /** Entry point in the kernel to start at */
    Addr kernelEntry;

    Kernel::Binning *kernelBinning;

#else

    int page_ptr;


#endif // FULL_SYSTEM

  protected:

#if FULL_SYSTEM
    /**
     * Fix up an address used to match PCs for hooking simulator
     * events on to target function executions.  See comment in
     * system.cc for details.
     */
    virtual Addr fixFuncEventAddr(Addr addr) = 0;

    /**
     * Add a function-based event to the given function, to be looked
     * up in the specified symbol table.
     */
    template <class T>
    T *System::addFuncEvent(SymbolTable *symtab, const char *lbl)
    {
        Addr addr = 0; // initialize only to avoid compiler warning

        if (symtab->findAddress(lbl, addr)) {
            T *ev = new T(&pcEventQueue, lbl, fixFuncEventAddr(addr));
            return ev;
        }

        return NULL;
    }

    /** Add a function-based event to kernel code. */
    template <class T>
    T *System::addKernelFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(kernelSymtab, lbl);
    }

#endif
  public:
#if FULL_SYSTEM
    std::vector<RemoteGDB *> remoteGDB;
    std::vector<GDBListener *> gdbListen;
    virtual bool breakpoint() = 0;
#endif // FULL_SYSTEM

  public:
    struct Params
    {
        std::string name;
        MemObject *physmem;

#if FULL_SYSTEM
        Tick boot_cpu_frequency;
        MemoryController *memctrl;
        uint64_t init_param;
        bool bin;
        std::vector<std::string> binned_fns;
        bool bin_int;

        std::string kernel_path;
        std::string readfile;
#endif
    };

  protected:
    Params *_params;

  public:
    System(Params *p);
    ~System();

    void startup();

    const Params *params() const { return (const Params *)_params; }

  public:

#if FULL_SYSTEM
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

#else

    Addr new_page();

#endif // FULL_SYSTEM

    int registerExecContext(ExecContext *xc, int xcIndex);
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
