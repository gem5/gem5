/*
 * Copyright (c) 2011 ARM Limited
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

#include "arch/interrupts.hh"
#include "arch/isa_traits.hh"
#include "arch/microcode_rom.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "mem/mem_object.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/insttracer.hh"

struct BaseCPUParams;
class BranchPred;
class CheckerCPU;
class ThreadContext;
class System;

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

    // @todo remove me after debugging with legion done
    Tick instCnt;
    // every cpu has an id, put it in the base cpu
    // Set at initialization, only time a cpuId might change is during a
    // takeover (which should be done from within the BaseCPU anyway,
    // therefore no setCpuId() method is provided
    int _cpuId;

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

    /**
     * Define a base class for the CPU ports (instruction and data)
     * that is refined in the subclasses. This class handles the
     * common cases, i.e. the functional accesses and the status
     * changes and address range queries. The default behaviour for
     * both atomic and timing access is to panic and the corresponding
     * subclasses have to override these methods.
     */
    class CpuPort : public MasterPort
    {
      public:

        /**
         * Create a CPU port with a name and a structural owner.
         *
         * @param _name port name including the owner
         * @param _name structural owner of this port
         */
        CpuPort(const std::string& _name, MemObject* _owner) :
            MasterPort(_name, _owner)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvRetry();

        virtual void recvFunctionalSnoop(PacketPtr pkt);

    };

  public:

    /**
     * Purely virtual method that returns a reference to the data
     * port. All subclasses must implement this method.
     *
     * @return a reference to the data port
     */
    virtual CpuPort &getDataPort() = 0;

    /**
     * Purely virtual method that returns a reference to the instruction
     * port. All subclasses must implement this method.
     *
     * @return a reference to the instruction port
     */
    virtual CpuPort &getInstPort() = 0;

    /** Reads this CPU's ID. */
    int cpuId() { return _cpuId; }

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
                                  PortID idx = InvalidPortID);

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
    TheISA::Interrupts *interrupts;

  public:
    TheISA::Interrupts *
    getInterruptController()
    {
        return interrupts;
    }

    virtual void wakeup() = 0;

    void
    postInterrupt(int int_num, int index)
    {
        interrupts->post(int_num, index);
        if (FullSystem)
            wakeup();
    }

    void
    clearInterrupt(int int_num, int index)
    {
        interrupts->clear(int_num, index);
    }

    void
    clearInterrupts()
    {
        interrupts->clearAll();
    }

    bool
    checkInterrupts(ThreadContext *tc) const
    {
        return FullSystem && interrupts->checkInterrupts(tc);
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

    // Mask to align PCs to MachInst sized boundaries
    static const Addr PCMask = ~((Addr)sizeof(TheISA::MachInst) - 1);

    /// Provide access to the tracer pointer
    Trace::InstTracer * getTracer() { return tracer; }

    /// Notify the CPU that the indicated context is now active.  The
    /// delay parameter indicates the number of ticks to wait before
    /// executing (typically 0 or 1).
    virtual void activateContext(ThreadID thread_num, Cycles delay) {}

    /// Notify the CPU that the indicated context is now suspended.
    virtual void suspendContext(ThreadID thread_num) {}

    /// Notify the CPU that the indicated context is now deallocated.
    virtual void deallocateContext(ThreadID thread_num) {}

    /// Notify the CPU that the indicated context is now halted.
    virtual void haltContext(ThreadID thread_num) {}

   /// Given a Thread Context pointer return the thread num
   int findContext(ThreadContext *tc);

   /// Given a thread num get tho thread context for it
   ThreadContext *getContext(int tn) { return threadContexts[tn]; }

  public:
    typedef BaseCPUParams Params;
    const Params *params() const
    { return reinterpret_cast<const Params *>(_params); }
    BaseCPU(Params *params, bool is_checker = false);
    virtual ~BaseCPU();

    virtual void init();
    virtual void startup();
    virtual void regStats();

    virtual void activateWhenReady(ThreadID tid) {};

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
     * Serialize this object to the given output stream.
     * @param os The stream to serialize to.
     */
    virtual void serialize(std::ostream &os);

    /**
     * Reconstruct the state of this object from a checkpoint.
     * @param cp The checkpoint use.
     * @param section The section name of this object
     */
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    /**
     * Return pointer to CPU's branch predictor (NULL if none).
     * @return Branch predictor pointer.
     */
    virtual BranchPred *getBranchPred() { return NULL; };

    virtual Counter totalInsts() const = 0;

    virtual Counter totalOps() const = 0;

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
};

#endif // __CPU_BASE_HH__
