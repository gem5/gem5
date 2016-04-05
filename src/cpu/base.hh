/*
 * Copyright (c) 2011-2013 ARM Limited
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
 *          Nathan Binkert
 *          Rick Strong
 */

#ifndef __CPU_BASE_HH__
#define __CPU_BASE_HH__

#include <vector>

// Before we do anything else, check if this build is the NULL ISA,
// and if so stop here
#include "config/the_isa.hh"
#if THE_ISA == NULL_ISA
#include "arch/null/cpu_dummy.hh"
#else
#include "arch/interrupts.hh"
#include "arch/isa_traits.hh"
#include "arch/microcode_rom.hh"
#include "base/statistics.hh"
#include "mem/mem_object.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/insttracer.hh"
#include "sim/probe/pmu.hh"
#include "sim/system.hh"
#include "debug/Mwait.hh"

class BaseCPU;
struct BaseCPUParams;
class CheckerCPU;
class ThreadContext;

struct AddressMonitor
{
    AddressMonitor();
    bool doMonitor(PacketPtr pkt);

    bool armed;
    Addr vAddr;
    Addr pAddr;
    uint64_t val;
    bool waiting;   // 0=normal, 1=mwaiting
    bool gotWakeup;
};

class CPUProgressEvent : public Event
{
  protected:
    Tick _interval;
    Counter lastNumInst;
    BaseCPU *cpu;
    bool _repeatEvent;

  public:
    CPUProgressEvent(BaseCPU *_cpu, Tick ival = 0);

    void process();

    void interval(Tick ival) { _interval = ival; }
    Tick interval() { return _interval; }

    void repeatEvent(bool repeat) { _repeatEvent = repeat; }

    virtual const char *description() const;
};

class BaseCPU : public MemObject
{
  protected:

    /// Instruction count used for SPARC misc register
    /// @todo unify this with the counters that cpus individually keep
    Tick instCnt;

    // every cpu has an id, put it in the base cpu
    // Set at initialization, only time a cpuId might change is during a
    // takeover (which should be done from within the BaseCPU anyway,
    // therefore no setCpuId() method is provided
    int _cpuId;

    /** Each cpu will have a socket ID that corresponds to its physical location
     * in the system. This is usually used to bucket cpu cores under single DVFS
     * domain. This information may also be required by the OS to identify the
     * cpu core grouping (as in the case of ARM via MPIDR register)
     */
    const uint32_t _socketId;

    /** instruction side request id that must be placed in all requests */
    MasterID _instMasterId;

    /** data side request id that must be placed in all requests */
    MasterID _dataMasterId;

    /** An intrenal representation of a task identifier within gem5. This is
     * used so the CPU can add which taskId (which is an internal representation
     * of the OS process ID) to each request so components in the memory system
     * can track which process IDs are ultimately interacting with them
     */
    uint32_t _taskId;

    /** The current OS process ID that is executing on this processor. This is
     * used to generate a taskId */
    uint32_t _pid;

    /** Is the CPU switched out or active? */
    bool _switchedOut;

    /** Cache the cache line size that we get from the system */
    const unsigned int _cacheLineSize;

  public:

    /**
     * Purely virtual method that returns a reference to the data
     * port. All subclasses must implement this method.
     *
     * @return a reference to the data port
     */
    virtual MasterPort &getDataPort() = 0;

    /**
     * Purely virtual method that returns a reference to the instruction
     * port. All subclasses must implement this method.
     *
     * @return a reference to the instruction port
     */
    virtual MasterPort &getInstPort() = 0;

    /** Reads this CPU's ID. */
    int cpuId() const { return _cpuId; }

    /** Reads this CPU's Socket ID. */
    uint32_t socketId() const { return _socketId; }

    /** Reads this CPU's unique data requestor ID */
    MasterID dataMasterId() { return _dataMasterId; }
    /** Reads this CPU's unique instruction requestor ID */
    MasterID instMasterId() { return _instMasterId; }

    /**
     * Get a master port on this CPU. All CPUs have a data and
     * instruction port, and this method uses getDataPort and
     * getInstPort of the subclasses to resolve the two ports.
     *
     * @param if_name the port name
     * @param idx ignored index
     *
     * @return a reference to the port with the given name
     */
    BaseMasterPort &getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    /** Get cpu task id */
    uint32_t taskId() const { return _taskId; }
    /** Set cpu task id */
    void taskId(uint32_t id) { _taskId = id; }

    uint32_t getPid() const { return _pid; }
    void setPid(uint32_t pid) { _pid = pid; }

    inline void workItemBegin() { numWorkItemsStarted++; }
    inline void workItemEnd() { numWorkItemsCompleted++; }
    // @todo remove me after debugging with legion done
    Tick instCount() { return instCnt; }

    TheISA::MicrocodeRom microcodeRom;

  protected:
    std::vector<TheISA::Interrupts*> interrupts;

  public:
    TheISA::Interrupts *
    getInterruptController(ThreadID tid)
    {
        if (interrupts.empty())
            return NULL;

        assert(interrupts.size() > tid);
        return interrupts[tid];
    }

    virtual void wakeup(ThreadID tid) = 0;

    void
    postInterrupt(ThreadID tid, int int_num, int index)
    {
        interrupts[tid]->post(int_num, index);
        if (FullSystem)
            wakeup(tid);
    }

    void
    clearInterrupt(ThreadID tid, int int_num, int index)
    {
        interrupts[tid]->clear(int_num, index);
    }

    void
    clearInterrupts(ThreadID tid)
    {
        interrupts[tid]->clearAll();
    }

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        return FullSystem && interrupts[tc->threadId()]->checkInterrupts(tc);
    }

    class ProfileEvent : public Event
    {
      private:
        BaseCPU *cpu;
        Tick interval;

      public:
        ProfileEvent(BaseCPU *cpu, Tick interval);
        void process();
    };
    ProfileEvent *profileEvent;

  protected:
    std::vector<ThreadContext *> threadContexts;

    Trace::InstTracer * tracer;

  public:


    /** Invalid or unknown Pid. Possible when operating system is not present
     *  or has not assigned a pid yet */
    static const uint32_t invldPid = std::numeric_limits<uint32_t>::max();

    // Mask to align PCs to MachInst sized boundaries
    static const Addr PCMask = ~((Addr)sizeof(TheISA::MachInst) - 1);

    /// Provide access to the tracer pointer
    Trace::InstTracer * getTracer() { return tracer; }

    /// Notify the CPU that the indicated context is now active.
    virtual void activateContext(ThreadID thread_num);

    /// Notify the CPU that the indicated context is now suspended.
    /// Check if possible to enter a lower power state
    virtual void suspendContext(ThreadID thread_num);

    /// Notify the CPU that the indicated context is now halted.
    virtual void haltContext(ThreadID thread_num) {}

   /// Given a Thread Context pointer return the thread num
   int findContext(ThreadContext *tc);

   /// Given a thread num get tho thread context for it
   virtual ThreadContext *getContext(int tn) { return threadContexts[tn]; }

   /// Get the number of thread contexts available
   unsigned numContexts() { return threadContexts.size(); }

    /// Convert ContextID to threadID
    ThreadID contextToThread(ContextID cid)
    { return static_cast<ThreadID>(cid - threadContexts[0]->contextId()); }

  public:
    typedef BaseCPUParams Params;
    const Params *params() const
    { return reinterpret_cast<const Params *>(_params); }
    BaseCPU(Params *params, bool is_checker = false);
    virtual ~BaseCPU();

    void init() override;
    void startup() override;
    void regStats() override;

    void regProbePoints() override;

    void registerThreadContexts();

    /**
     * Prepare for another CPU to take over execution.
     *
     * When this method exits, all internal state should have been
     * flushed. After the method returns, the simulator calls
     * takeOverFrom() on the new CPU with this CPU as its parameter.
     */
    virtual void switchOut();

    /**
     * Load the state of a CPU from the previous CPU object, invoked
     * on all new CPUs that are about to be switched in.
     *
     * A CPU model implementing this method is expected to initialize
     * its state from the old CPU and connect its memory (unless they
     * are already connected) to the memories connected to the old
     * CPU.
     *
     * @param cpu CPU to initialize read state from.
     */
    virtual void takeOverFrom(BaseCPU *cpu);

    /**
     * Flush all TLBs in the CPU.
     *
     * This method is mainly used to flush stale translations when
     * switching CPUs. It is also exported to the Python world to
     * allow it to request a TLB flush after draining the CPU to make
     * it easier to compare traces when debugging
     * handover/checkpointing.
     */
    void flushTLBs();

    /**
     * Determine if the CPU is switched out.
     *
     * @return True if the CPU is switched out, false otherwise.
     */
    bool switchedOut() const { return _switchedOut; }

    /**
     * Verify that the system is in a memory mode supported by the
     * CPU.
     *
     * Implementations are expected to query the system for the
     * current memory mode and ensure that it is what the CPU model
     * expects. If the check fails, the implementation should
     * terminate the simulation using fatal().
     */
    virtual void verifyMemoryMode() const { };

    /**
     *  Number of threads we're actually simulating (<= SMT_MAX_THREADS).
     * This is a constant for the duration of the simulation.
     */
    ThreadID numThreads;

    /**
     * Vector of per-thread instruction-based event queues.  Used for
     * scheduling events based on number of instructions committed by
     * a particular thread.
     */
    EventQueue **comInstEventQueue;

    /**
     * Vector of per-thread load-based event queues.  Used for
     * scheduling events based on number of loads committed by
     *a particular thread.
     */
    EventQueue **comLoadEventQueue;

    System *system;

    /**
     * Get the cache line size of the system.
     */
    inline unsigned int cacheLineSize() const { return _cacheLineSize; }

    /**
     * Serialize this object to the given output stream.
     *
     * @note CPU models should normally overload the serializeThread()
     * method instead of the serialize() method as this provides a
     * uniform data format for all CPU models and promotes better code
     * reuse.
     *
     * @param os The stream to serialize to.
     */
    void serialize(CheckpointOut &cp) const override;

    /**
     * Reconstruct the state of this object from a checkpoint.
     *
     * @note CPU models should normally overload the
     * unserializeThread() method instead of the unserialize() method
     * as this provides a uniform data format for all CPU models and
     * promotes better code reuse.

     * @param cp The checkpoint use.
     * @param section The section name of this object.
     */
    void unserialize(CheckpointIn &cp) override;

    /**
     * Serialize a single thread.
     *
     * @param os The stream to serialize to.
     * @param tid ID of the current thread.
     */
    virtual void serializeThread(CheckpointOut &cp, ThreadID tid) const {};

    /**
     * Unserialize one thread.
     *
     * @param cp The checkpoint use.
     * @param section The section name of this thread.
     * @param tid ID of the current thread.
     */
    virtual void unserializeThread(CheckpointIn &cp, ThreadID tid) {};

    virtual Counter totalInsts() const = 0;

    virtual Counter totalOps() const = 0;

    /**
     * Schedule an event that exits the simulation loops after a
     * predefined number of instructions.
     *
     * This method is usually called from the configuration script to
     * get an exit event some time in the future. It is typically used
     * when the script wants to simulate for a specific number of
     * instructions rather than ticks.
     *
     * @param tid Thread monitor.
     * @param insts Number of instructions into the future.
     * @param cause Cause to signal in the exit event.
     */
    void scheduleInstStop(ThreadID tid, Counter insts, const char *cause);

    /**
     * Schedule an event that exits the simulation loops after a
     * predefined number of load operations.
     *
     * This method is usually called from the configuration script to
     * get an exit event some time in the future. It is typically used
     * when the script wants to simulate for a specific number of
     * loads rather than ticks.
     *
     * @param tid Thread monitor.
     * @param loads Number of load instructions into the future.
     * @param cause Cause to signal in the exit event.
     */
    void scheduleLoadStop(ThreadID tid, Counter loads, const char *cause);

    /**
     * Get the number of instructions executed by the specified thread
     * on this CPU. Used by Python to control simulation.
     *
     * @param tid Thread monitor
     * @return Number of instructions executed
     */
    uint64_t getCurrentInstCount(ThreadID tid);

  public:
    /**
     * @{
     * @name PMU Probe points.
     */

    /**
     * Helper method to trigger PMU probes for a committed
     * instruction.
     *
     * @param inst Instruction that just committed
     */
    virtual void probeInstCommit(const StaticInstPtr &inst);

    /**
     * Helper method to instantiate probe points belonging to this
     * object.
     *
     * @param name Name of the probe point.
     * @return A unique_ptr to the new probe point.
     */
    ProbePoints::PMUUPtr pmuProbePoint(const char *name);

    /** CPU cycle counter */
    ProbePoints::PMUUPtr ppCycles;

    /**
     * Instruction commit probe point.
     *
     * This probe point is triggered whenever one or more instructions
     * are committed. It is normally triggered once for every
     * instruction. However, CPU models committing bundles of
     * instructions may call notify once for the entire bundle.
     */
    ProbePoints::PMUUPtr ppRetiredInsts;

    /** Retired load instructions */
    ProbePoints::PMUUPtr ppRetiredLoads;
    /** Retired store instructions */
    ProbePoints::PMUUPtr ppRetiredStores;

    /** Retired branches (any type) */
    ProbePoints::PMUUPtr ppRetiredBranches;

    /** @} */



    // Function tracing
  private:
    bool functionTracingEnabled;
    std::ostream *functionTraceStream;
    Addr currentFunctionStart;
    Addr currentFunctionEnd;
    Tick functionEntryTick;
    void enableFunctionTrace();
    void traceFunctionsInternal(Addr pc);

  private:
    static std::vector<BaseCPU *> cpuList;   //!< Static global cpu list

  public:
    void traceFunctions(Addr pc)
    {
        if (functionTracingEnabled)
            traceFunctionsInternal(pc);
    }

    static int numSimulatedCPUs() { return cpuList.size(); }
    static Counter numSimulatedInsts()
    {
        Counter total = 0;

        int size = cpuList.size();
        for (int i = 0; i < size; ++i)
            total += cpuList[i]->totalInsts();

        return total;
    }

    static Counter numSimulatedOps()
    {
        Counter total = 0;

        int size = cpuList.size();
        for (int i = 0; i < size; ++i)
            total += cpuList[i]->totalOps();

        return total;
    }

  public:
    // Number of CPU cycles simulated
    Stats::Scalar numCycles;
    Stats::Scalar numWorkItemsStarted;
    Stats::Scalar numWorkItemsCompleted;

  private:
    std::vector<AddressMonitor> addressMonitor;

  public:
    void armMonitor(ThreadID tid, Addr address);
    bool mwait(ThreadID tid, PacketPtr pkt);
    void mwaitAtomic(ThreadID tid, ThreadContext *tc, TheISA::TLB *dtb);
    AddressMonitor *getCpuAddrMonitor(ThreadID tid)
    {
        assert(tid < numThreads);
        return &addressMonitor[tid];
    }
};

#endif // THE_ISA == NULL_ISA

#endif // __CPU_BASE_HH__
