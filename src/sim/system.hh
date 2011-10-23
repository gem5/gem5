/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2011 Regents of the University of California
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
 * Authors: Steve Reinhardt
 *          Lisa Hsu
 *          Nathan Binkert
 *          Rick Strong
 */

#ifndef __SYSTEM_HH__
#define __SYSTEM_HH__

#include <string>
#include <vector>

#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/statistics.hh"
#include "config/full_system.hh"
#include "cpu/pc_event.hh"
#include "enums/MemoryMode.hh"
#include "mem/port.hh"
#include "params/System.hh"
#include "sim/sim_object.hh"

#if FULL_SYSTEM
#include "kern/system_events.hh"
#endif

class BaseCPU;
class ThreadContext;
class ObjectFile;
class PhysicalMemory;

#if FULL_SYSTEM
class Platform;
class FunctionalPort;
class VirtualPort;
#endif
class GDBListener;
class BaseRemoteGDB;

class System : public SimObject
{
  public:

    static const char *MemoryModeStrings[3];

    Enums::MemoryMode
    getMemoryMode()
    {
        assert(memoryMode);
        return memoryMode;
    }

    /** Change the memory mode of the system. This should only be called by the
     * python!!
     * @param mode Mode to change to (atomic/timing)
     */
    void setMemoryMode(Enums::MemoryMode mode);

    PhysicalMemory *physmem;
    PCEventQueue pcEventQueue;

    std::vector<ThreadContext *> threadContexts;
    int _numContexts;

    ThreadContext *getThreadContext(ThreadID tid)
    {
        return threadContexts[tid];
    }

    int numContexts()
    {
        assert(_numContexts == (int)threadContexts.size());
        return _numContexts;
    }

    /** Return number of running (non-halted) thread contexts in
     * system.  These threads could be Active or Suspended. */
    int numRunningContexts();

    /** List to store ranges of memories in this system */
    AddrRangeList memRanges;

    /** check if an address points to valid system memory
     * and thus we can fetch instructions out of it
     */
    bool isMemory(const Addr addr) const;

#if FULL_SYSTEM
    Platform *platform;
    uint64_t init_param;

    /** Port to physical memory used for writing object files into ram at
     * boot.*/
    FunctionalPort *functionalPort;
    VirtualPort *virtPort;

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

    /** Mask that should be anded for binary/symbol loading.
     * This allows one two different OS requirements for the same ISA to be
     * handled.  Some OSes are compiled for a virtual address and need to be
     * loaded into physical memory that starts at address 0, while other
     * bare metal tools generate images that start at address 0.
     */
    Addr loadAddrMask;

#else

    Addr pagePtr;

  protected:
    uint64_t nextPID;

  public:
    uint64_t allocatePID()
    {
        return nextPID++;
    }

    /** Amount of physical memory that is still free */
    Addr freeMemSize();

    /** Amount of physical memory that exists */
    Addr memSize();


#endif // FULL_SYSTEM

  protected:
    Enums::MemoryMode memoryMode;
    uint64_t workItemsBegin;
    uint64_t workItemsEnd;
    std::vector<bool> activeCpus;

  public:
    /**
     * Called by pseudo_inst to track the number of work items started by this
     * system.
     */
    uint64_t 
    incWorkItemsBegin()
    {
        return ++workItemsBegin;
    }

    /**
     * Called by pseudo_inst to track the number of work items completed by
     * this system.
     */
    uint64_t 
    incWorkItemsEnd()
    {
        return ++workItemsEnd;
    }

    /**
     * Called by pseudo_inst to mark the cpus actively executing work items.
     * Returns the total number of cpus that have executed work item begin or
     * ends.
     */
    int 
    markWorkItem(int index)
    {
        int count = 0;
        assert(index < activeCpus.size());
        activeCpus[index] = true;
        for (std::vector<bool>::iterator i = activeCpus.begin(); 
             i < activeCpus.end(); i++) {
            if (*i) count++;
        }
        return count;
    }

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
    T *addFuncEvent(SymbolTable *symtab, const char *lbl)
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
    T *addKernelFuncEvent(const char *lbl)
    {
        return addFuncEvent<T>(kernelSymtab, lbl);
    }

#endif
  public:
    std::vector<BaseRemoteGDB *> remoteGDB;
    std::vector<GDBListener *> gdbListen;
    bool breakpoint();

  public:
    typedef SystemParams Params;

  protected:
    Params *_params;

  public:
    System(Params *p);
    ~System();

    void initState();

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

    /// Allocate npages contiguous unused physical pages
    /// @return Starting address of first page
    Addr allocPhysPages(int npages);

#endif // FULL_SYSTEM

    int registerThreadContext(ThreadContext *tc, int assigned=-1);
    void replaceThreadContext(ThreadContext *tc, int context_id);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
    virtual void resume();

  public:
    Counter totalNumInsts;
    EventQueue instEventQueue;

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
