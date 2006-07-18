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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __CPU_BASE_HH__
#define __CPU_BASE_HH__

#include <vector>

#include "base/statistics.hh"
#include "config/full_system.hh"
#include "sim/eventq.hh"
#include "mem/mem_object.hh"
#include "arch/isa_traits.hh"

class BranchPred;
class CheckerCPU;
class ThreadContext;
class System;
class Port;

class BaseCPU : public MemObject
{
  protected:
    // CPU's clock period in terms of the number of ticks of curTime.
    Tick clock;

  public:
    inline Tick frequency() const { return Clock::Frequency / clock; }
    inline Tick cycles(int numCycles) const { return clock * numCycles; }
    inline Tick curCycle() const { return curTick / clock; }

#if FULL_SYSTEM
  protected:
    uint64_t interrupts[TheISA::NumInterruptLevels];
    uint64_t intstatus;

  public:
    virtual void post_interrupt(int int_num, int index);
    virtual void clear_interrupt(int int_num, int index);
    virtual void clear_interrupts();
    bool checkInterrupts;

    bool check_interrupt(int int_num) const {
        if (int_num > TheISA::NumInterruptLevels)
            panic("int_num out of bounds\n");

        return interrupts[int_num] != 0;
    }

    bool check_interrupts() const { return intstatus != 0; }
    uint64_t intr_status() const { return intstatus; }

    class ProfileEvent : public Event
    {
      private:
        BaseCPU *cpu;
        int interval;

      public:
        ProfileEvent(BaseCPU *cpu, int interval);
        void process();
    };
    ProfileEvent *profileEvent;
#endif

  protected:
    std::vector<ThreadContext *> threadContexts;

  public:

    /// Notify the CPU that the indicated context is now active.  The
    /// delay parameter indicates the number of ticks to wait before
    /// executing (typically 0 or 1).
    virtual void activateContext(int thread_num, int delay) {}

    /// Notify the CPU that the indicated context is now suspended.
    virtual void suspendContext(int thread_num) {}

    /// Notify the CPU that the indicated context is now deallocated.
    virtual void deallocateContext(int thread_num) {}

    /// Notify the CPU that the indicated context is now halted.
    virtual void haltContext(int thread_num) {}

  public:
    struct Params
    {
        std::string name;
        int numberOfThreads;
        bool deferRegistration;
        Counter max_insts_any_thread;
        Counter max_insts_all_threads;
        Counter max_loads_any_thread;
        Counter max_loads_all_threads;
        Tick clock;
        bool functionTrace;
        Tick functionTraceStart;
        System *system;
#if FULL_SYSTEM
        int cpu_id;
        Tick profile;
#endif
        BaseCPU *checker;

        Params();
    };

    const Params *params;

    BaseCPU(Params *params);
    virtual ~BaseCPU();

    virtual void init();
    virtual void startup();
    virtual void regStats();

    virtual void activateWhenReady(int tid) {};

    void registerThreadContexts();

    /// Prepare for another CPU to take over execution.  When it is
    /// is ready (drained pipe) it signals the sampler.
    virtual void switchOut();

    /// Take over execution from the given CPU.  Used for warm-up and
    /// sampling.
    virtual void takeOverFrom(BaseCPU *);

    /**
     *  Number of threads we're actually simulating (<= SMT_MAX_THREADS).
     * This is a constant for the duration of the simulation.
     */
    int number_of_threads;

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

#if FULL_SYSTEM
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

#endif

    /**
     * Return pointer to CPU's branch predictor (NULL if none).
     * @return Branch predictor pointer.
     */
    virtual BranchPred *getBranchPred() { return NULL; };

    virtual Counter totalInstructions() const { return 0; }

    // Function tracing
  private:
    bool functionTracingEnabled;
    std::ostream *functionTraceStream;
    Addr currentFunctionStart;
    Addr currentFunctionEnd;
    Tick functionEntryTick;
    void enableFunctionTrace();
    void traceFunctionsInternal(Addr pc);

  protected:
    void traceFunctions(Addr pc)
    {
        if (functionTracingEnabled)
            traceFunctionsInternal(pc);
    }

  private:
    static std::vector<BaseCPU *> cpuList;   //!< Static global cpu list

  public:
    static int numSimulatedCPUs() { return cpuList.size(); }
    static Counter numSimulatedInstructions()
    {
        Counter total = 0;

        int size = cpuList.size();
        for (int i = 0; i < size; ++i)
            total += cpuList[i]->totalInstructions();

        return total;
    }

  public:
    // Number of CPU cycles simulated
    Stats::Scalar<> numCycles;
};

#endif // __CPU_BASE_HH__
