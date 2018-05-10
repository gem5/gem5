/*
 * Copyright (c) 2012, 2014, 2018 ARM Limited
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
#include <unordered_map>
#include <utility>
#include <vector>

#include "arch/isa_traits.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "enums/MemoryMode.hh"
#include "mem/mem_master.hh"
#include "mem/mem_object.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "params/System.hh"
#include "sim/futex_map.hh"
#include "sim/se_signal.hh"

/**
 * To avoid linking errors with LTO, only include the header if we
 * actually have the definition.
 */
#if THE_ISA != NULL_ISA
#include "cpu/pc_event.hh"

#endif

class BaseRemoteGDB;
class KvmVM;
class ObjectFile;
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
        bool recvTimingResp(PacketPtr pkt) override
        { panic("SystemPort does not receive timing!\n"); return false; }
        void recvReqRetry() override
        { panic("SystemPort does not expect retry!\n"); }
    };

    SystemPort _systemPort;

  public:

    /**
     * After all objects have been created and all ports are
     * connected, check that the system port is connected.
     */
    void init() override;

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
    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    /** @{ */
    /**
     * Is the system in atomic mode?
     *
     * There are currently two different atomic memory modes:
     * 'atomic', which supports caches; and 'atomic_noncaching', which
     * bypasses caches. The latter is used by hardware virtualized
     * CPUs. SimObjects are expected to use Port::sendAtomic() and
     * Port::recvAtomic() when accessing memory in this mode.
     */
    bool isAtomicMode() const {
        return memoryMode == Enums::atomic ||
            memoryMode == Enums::atomic_noncaching;
    }

    /**
     * Is the system in timing mode?
     *
     * SimObjects are expected to use Port::sendTiming() and
     * Port::recvTiming() when accessing memory in this mode.
     */
    bool isTimingMode() const {
        return memoryMode == Enums::timing;
    }

    /**
     * Should caches be bypassed?
     *
     * Some CPUs need to bypass caches to allow direct memory
     * accesses, which is required for hardware virtualization.
     */
    bool bypassCaches() const {
        return memoryMode == Enums::atomic_noncaching;
    }
    /** @} */

    /** @{ */
    /**
     * Get the memory mode of the system.
     *
     * \warn This should only be used by the Python world. The C++
     * world should use one of the query functions above
     * (isAtomicMode(), isTimingMode(), bypassCaches()).
     */
    Enums::MemoryMode getMemoryMode() const { return memoryMode; }

    /**
     * Change the memory mode of the system.
     *
     * \warn This should only be called by the Python!
     *
     * @param mode Mode to change to (atomic/timing/...)
     */
    void setMemoryMode(Enums::MemoryMode mode);
    /** @} */

    /**
     * Get the cache line size of the system.
     */
    unsigned int cacheLineSize() const { return _cacheLineSize; }

#if THE_ISA != NULL_ISA
    PCEventQueue pcEventQueue;
#endif

    std::vector<ThreadContext *> threadContexts;
    const bool multiThread;

    ThreadContext *getThreadContext(ContextID tid)
    {
        return threadContexts[tid];
    }

    unsigned numContexts() const { return threadContexts.size(); }

    /** Return number of running (non-halted) thread contexts in
     * system.  These threads could be Active or Suspended. */
    int numRunningContexts();

    Addr pagePtr;

    uint64_t init_param;

    /** Port to physical memory used for writing object files into ram at
     * boot.*/
    PortProxy physProxy;

    /** kernel symbol table */
    SymbolTable *kernelSymtab;

    /** Object pointer for the kernel code */
    ObjectFile *kernel;

    /** Additional object files */
    std::vector<ObjectFile *> kernelExtras;

    /** Beginning of kernel code */
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

    /** Offset that should be used for binary/symbol loading.
     * This further allows more flexibility than the loadAddrMask allows alone
     * in loading kernels and similar. The loadAddrOffset is applied after the
     * loadAddrMask.
     */
    Addr loadAddrOffset;

  public:
    /**
     * Get a pointer to the Kernel Virtual Machine (KVM) SimObject,
     * if present.
     */
    KvmVM* getKvmVM() {
        return kvmVM;
    }

    /** Verify gem5 configuration will support KVM emulation */
    bool validKvmEnvironment() const;

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

    /**
     * Get the architecture.
     */
    Arch getArch() const { return Arch::TheISA; }

     /**
     * Get the page bytes for the ISA.
     */
    Addr getPageBytes() const { return TheISA::PageBytes; }

    /**
     * Get the number of bits worth of in-page address for the ISA.
     */
    Addr getPageShift() const { return TheISA::PageShift; }

    /**
     * The thermal model used for this system (if any).
     */
    ThermalModel * getThermalModel() const { return thermalModel; }

  protected:

    KvmVM *const kvmVM;

    PhysicalMemory physmem;

    Enums::MemoryMode memoryMode;

    const unsigned int _cacheLineSize;

    uint64_t workItemsBegin;
    uint64_t workItemsEnd;
    uint32_t numWorkIds;
    std::vector<bool> activeCpus;

    /** This array is a per-system list of all devices capable of issuing a
     * memory system request and an associated string for each master id.
     * It's used to uniquely id any master in the system by name for things
     * like cache statistics.
     */
    std::vector<MasterInfo> masters;

    ThermalModel * thermalModel;

  protected:
    /**
     * Strips off the system name from a master name
     */
    std::string stripSystemName(const std::string& master_name) const;

  public:

    /**
     * Request an id used to create a request object in the system. All objects
     * that intend to issues requests into the memory system must request an id
     * in the init() phase of startup. All master ids must be fixed by the
     * regStats() phase that immediately precedes it. This allows objects in
     * the memory system to understand how many masters may exist and
     * appropriately name the bins of their per-master stats before the stats
     * are finalized.
     *
     * Registers a MasterID:
     * This method takes two parameters, one of which is optional.
     * The first one is the master object, and it is compulsory; in case
     * a object has multiple (sub)masters, a second parameter must be
     * provided and it contains the name of the submaster. The method will
     * create a master's name by concatenating the SimObject name with the
     * eventual submaster string, separated by a dot.
     *
     * As an example:
     * For a cpu having two masters: a data master and an instruction master,
     * the method must be called twice:
     *
     * instMasterId = getMasterId(cpu, "inst");
     * dataMasterId = getMasterId(cpu, "data");
     *
     * and the masters' names will be:
     * - "cpu.inst"
     * - "cpu.data"
     *
     * @param master SimObject related to the master
     * @param submaster String containing the submaster's name
     * @return the master's ID.
     */
    MasterID getMasterId(const SimObject* master,
                         std::string submaster = std::string());

    /**
     * Registers a GLOBAL MasterID, which is a MasterID not related
     * to any particular SimObject; since no SimObject is passed,
     * the master gets registered by providing the full master name.
     *
     * @param masterName full name of the master
     * @return the master's ID.
     */
    MasterID getGlobalMasterId(const std::string& master_name);

    /**
     * Get the name of an object for a given request id.
     */
    std::string getMasterName(MasterID master_id);

    /**
     * Looks up the MasterID for a given SimObject
     * returns an invalid MasterID (invldMasterId) if not found.
     */
    MasterID lookupMasterId(const SimObject* obj) const;

    /**
     * Looks up the MasterID for a given object name string
     * returns an invalid MasterID (invldMasterId) if not found.
     */
    MasterID lookupMasterId(const std::string& name) const;

    /** Get the number of masters registered in the system */
    MasterID maxMasters() { return masters.size(); }

  protected:
    /** helper function for getMasterId */
    MasterID _getMasterId(const SimObject* master,
                          const std::string& master_name);

    /**
     * Helper function for constructing the full (sub)master name
     * by providing the root master and the relative submaster name.
     */
    std::string leafMasterName(const SimObject* master,
                               const std::string& submaster);

  public:

    void regStats() override;
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

    /** @{ */
    /**
     * Add a function-based event to the given function, to be looked
     * up in the specified symbol table.
     *
     * The ...OrPanic flavor of the method causes the simulator to
     * panic if the symbol can't be found.
     *
     * @param symtab Symbol table to use for look up.
     * @param lbl Function to hook the event to.
     * @param desc Description to be passed to the event.
     * @param args Arguments to be forwarded to the event constructor.
     */
    template <class T, typename... Args>
    T *addFuncEvent(const SymbolTable *symtab, const char *lbl,
                    const std::string &desc, Args... args)
    {
        Addr addr M5_VAR_USED = 0; // initialize only to avoid compiler warning

#if THE_ISA != NULL_ISA
        if (symtab->findAddress(lbl, addr)) {
            T *ev = new T(&pcEventQueue, desc, fixFuncEventAddr(addr),
                          std::forward<Args>(args)...);
            return ev;
        }
#endif

        return NULL;
    }

    template <class T>
    T *addFuncEvent(const SymbolTable *symtab, const char *lbl)
    {
        return addFuncEvent<T>(symtab, lbl, lbl);
    }

    template <class T, typename... Args>
    T *addFuncEventOrPanic(const SymbolTable *symtab, const char *lbl,
                           Args... args)
    {
        T *e(addFuncEvent<T>(symtab, lbl, std::forward<Args>(args)...));
        if (!e)
            panic("Failed to find symbol '%s'", lbl);
        return e;
    }
    /** @} */

    /** @{ */
    /**
     * Add a function-based event to a kernel symbol.
     *
     * These functions work like their addFuncEvent() and
     * addFuncEventOrPanic() counterparts. The only difference is that
     * they automatically use the kernel symbol table. All arguments
     * are forwarded to the underlying method.
     *
     * @see addFuncEvent()
     * @see addFuncEventOrPanic()
     *
     * @param lbl Function to hook the event to.
     * @param args Arguments to be passed to addFuncEvent
     */
    template <class T, typename... Args>
    T *addKernelFuncEvent(const char *lbl, Args... args)
    {
        return addFuncEvent<T>(kernelSymtab, lbl,
                               std::forward<Args>(args)...);
    }

    template <class T, typename... Args>
    T *addKernelFuncEventOrPanic(const char *lbl, Args... args)
    {
        T *e(addFuncEvent<T>(kernelSymtab, lbl,
                             std::forward<Args>(args)...));
        if (!e)
            panic("Failed to find kernel symbol '%s'", lbl);
        return e;
    }
    /** @} */

  public:
    std::vector<BaseRemoteGDB *> remoteGDB;
    bool breakpoint();

  public:
    typedef SystemParams Params;

  protected:
    Params *_params;

  public:
    System(Params *p);
    ~System();

    void initState() override;

    const Params *params() const { return (const Params *)_params; }

  public:

    /**
     * Returns the address the kernel starts at.
     * @return address the kernel starts at
     */
    Addr getKernelStart() const { return kernelStart; }

    /**
     * Returns the address the kernel ends at.
     * @return address the kernel ends at
     */
    Addr getKernelEnd() const { return kernelEnd; }

    /**
     * Returns the address the entry point to the kernel code.
     * @return entry point of the kernel code
     */
    Addr getKernelEntry() const { return kernelEntry; }

    /// Allocate npages contiguous unused physical pages
    /// @return Starting address of first page
    Addr allocPhysPages(int npages);

    ContextID registerThreadContext(ThreadContext *tc,
                                    ContextID assigned = InvalidContextID);
    void replaceThreadContext(ThreadContext *tc, ContextID context_id);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void drainResume() override;

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

    FutexMap futexMap;

    static const int maxPID = 32768;

    /** Process set to track which PIDs have already been allocated */
    std::set<int> PIDs;

    // By convention, all signals are owned by the receiving process. The
    // receiver will delete the signal upon reception.
    std::list<BasicSignal> signalList;

  protected:

    /**
     * If needed, serialize additional symbol table entries for a
     * specific subclass of this system. Currently this is used by
     * Alpha and MIPS.
     *
     * @param os stream to serialize to
     */
    virtual void serializeSymtab(CheckpointOut &os) const {}

    /**
     * If needed, unserialize additional symbol table entries for a
     * specific subclass of this system.
     *
     * @param cp checkpoint to unserialize from
     * @param section relevant section in the checkpoint
     */
    virtual void unserializeSymtab(CheckpointIn &cp) {}

};

void printSystems();

#endif // __SYSTEM_HH__
