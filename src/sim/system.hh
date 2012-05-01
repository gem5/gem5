/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
#include "cpu/pc_event.hh"
#include "enums/MemoryMode.hh"
#include "kern/system_events.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "mem/physical.hh"
#include "params/System.hh"

class BaseCPU;
class BaseRemoteGDB;
class GDBListener;
class ObjectFile;
class Platform;
class ThreadContext;

class System : public MemObject
{
  private:

    /**
     * Private class for the system port which is only used as a
     * master for debug access and for non-structural entities that do
     * not have a port of their own.
     */
    class SystemPort : public MasterPort
    {
      public:

        /**
         * Create a system port with a name and an owner.
         */
        SystemPort(const std::string &_name, MemObject *_owner)
            : MasterPort(_name, _owner)
        { }
        bool recvTimingResp(PacketPtr pkt)
        { panic("SystemPort does not receive timing!\n"); return false; }
        void recvRetry()
        { panic("SystemPort does not expect retry!\n"); }
    };

    SystemPort _systemPort;

  public:

    /**
     * After all objects have been created and all ports are
     * connected, check that the system port is connected.
     */
    virtual void init();

    /**
     * Get a reference to the system port that can be used by
     * non-structural simulation objects like processes or threads, or
     * external entities like loaders and debuggers, etc, to access
     * the memory system.
     *
     * @return a reference to the system port we own
     */
    MasterPort& getSystemPort() { return _systemPort; }

    /**
     * Additional function to return the Port of a memory object.
     */
    MasterPort& getMasterPort(const std::string &if_name, int idx = -1);

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

    Addr pagePtr;

    uint64_t init_param;

    /** Port to physical memory used for writing object files into ram at
     * boot.*/
    PortProxy physProxy;
    FSTranslatingPortProxy virtProxy;

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

  protected:
    uint64_t nextPID;

  public:
    uint64_t allocatePID()
    {
        return nextPID++;
    }

    /** Get a pointer to access the physical memory of the system */
    PhysicalMemory& getPhysMem() { return physmem; }

    /** Amount of physical memory that is still free */
    Addr freeMemSize() const;

    /** Amount of physical memory that exists */
    Addr memSize() const;

    /**
     * Check if a physical address is within a range of a memory that
     * is part of the global address map.
     *
     * @param addr A physical address
     * @return Whether the address corresponds to a memory
     */
    bool isMemAddr(Addr addr) const;

  protected:

    PhysicalMemory physmem;

    Enums::MemoryMode memoryMode;
    uint64_t workItemsBegin;
    uint64_t workItemsEnd;
    uint32_t numWorkIds;
    std::vector<bool> activeCpus;

    /** This array is a per-sytem list of all devices capable of issuing a
     * memory system request and an associated string for each master id.
     * It's used to uniquely id any master in the system by name for things
     * like cache statistics.
     */
    std::vector<std::string> masterIds;

  public:

    /** Request an id used to create a request object in the system. All objects
     * that intend to issues requests into the memory system must request an id
     * in the init() phase of startup. All master ids must be fixed by the
     * regStats() phase that immediately preceeds it. This allows objects in the
     * memory system to understand how many masters may exist and
     * appropriately name the bins of their per-master stats before the stats
     * are finalized
     */
    MasterID getMasterId(std::string req_name);

    /** Get the name of an object for a given request id.
     */
    std::string getMasterName(MasterID master_id);

    /** Get the number of masters registered in the system */
    MasterID maxMasters()
    {
        return masterIds.size();
    }

    virtual void regStats();
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

    inline void workItemBegin(uint32_t tid, uint32_t workid)
    {
        std::pair<uint32_t,uint32_t> p(tid, workid);
        lastWorkItemStarted[p] = curTick();
    }

    void workItemEnd(uint32_t tid, uint32_t workid);

    /**
     * Fix up an address used to match PCs for hooking simulator
     * events on to target function executions.  See comment in
     * system.cc for details.
     */
    virtual Addr fixFuncEventAddr(Addr addr)
    {
        panic("Base fixFuncEventAddr not implemented.\n");
    }

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

    /// Allocate npages contiguous unused physical pages
    /// @return Starting address of first page
    Addr allocPhysPages(int npages);

    int registerThreadContext(ThreadContext *tc, int assigned=-1);
    void replaceThreadContext(ThreadContext *tc, int context_id);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
    virtual void resume();

  public:
    Counter totalNumInsts;
    EventQueue instEventQueue;
    std::map<std::pair<uint32_t,uint32_t>, Tick>  lastWorkItemStarted;
    std::map<uint32_t, Stats::Histogram*> workItemStats;

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
