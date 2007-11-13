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

#include "arch/isa_traits.hh"
#include "base/statistics.hh"
#include "config/full_system.hh"
#include "sim/eventq.hh"
#include "sim/insttracer.hh"
#include "mem/mem_object.hh"

#if FULL_SYSTEM
#include "arch/interrupts.hh"
#endif

class BranchPred;
class CheckerCPU;
class ThreadContext;
class System;
class Port;

namespace TheISA
{
    class Predecoder;
}

class CPUProgressEvent : public Event
{
  protected:
    Tick interval;
    Counter lastNumInst;
    BaseCPU *cpu;

  public:
    CPUProgressEvent(EventQueue *q, Tick ival, BaseCPU *_cpu);

    void process();

    virtual const char *description();
};

class BaseCPU : public MemObject
{
  protected:
    // CPU's clock period in terms of the number of ticks of curTime.
    Tick clock;
    // @todo remove me after debugging with legion done
    Tick instCnt;

  public:
//    Tick currentTick;
    inline Tick frequency() const { return Clock::Frequency / clock; }
    inline Tick ticks(int numCycles) const { return clock * numCycles; }
    inline Tick curCycle() const { return curTick / clock; }
    inline Tick tickToCycles(Tick val) const { return val / clock; }
    // @todo remove me after debugging with legion done
    Tick instCount() { return instCnt; }

    /** The next cycle the CPU should be scheduled, given a cache
     * access or quiesce event returning on this cycle.  This function
     * may return curTick if the CPU should run on the current cycle.
     */
    Tick nextCycle();

    /** The next cycle the CPU should be scheduled, given a cache
     * access or quiesce event returning on the given Tick.  This
     * function may return curTick if the CPU should run on the
     * current cycle.
     * @param begin_tick The tick that the event is completing on.
     */
    Tick nextCycle(Tick begin_tick);

#if FULL_SYSTEM
  protected:
//    uint64_t interrupts[TheISA::NumInterruptLevels];
//    uint64_t intstatus;
    TheISA::Interrupts interrupts;

  public:
    virtual void post_interrupt(int int_num, int index);
    virtual void clear_interrupt(int int_num, int index);
    virtual void clear_interrupts();
    virtual uint64_t get_interrupts(int int_num);

    bool check_interrupts(ThreadContext * tc) const
    { return interrupts.check_interrupts(tc); }

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
    std::vector<TheISA::Predecoder *> predecoders;

    Trace::InstTracer * tracer;

  public:

    /// Provide access to the tracer pointer
    Trace::InstTracer * getTracer() { return tracer; }

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

   /// Given a Thread Context pointer return the thread num
   int findContext(ThreadContext *tc);

   /// Given a thread num get tho thread context for it
   ThreadContext *getContext(int tn) { return threadContexts[tn]; }

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
        int cpu_id;
        Trace::InstTracer * tracer;

        Tick phase;
#if FULL_SYSTEM
        Tick profile;

        bool do_statistics_insts;
        bool do_checkpoint_insts;
        bool do_quiesce;
#endif
        Tick progress_interval;
        BaseCPU *checker;

#if THE_ISA == MIPS_ISA
      /* Note: It looks like it will be better to allow simulator users
         to specify the values of individual variables instead of requiring
         users to define the values of entire registers
         Especially since a lot of these variables can be created from other
         user parameters  (cache descriptions)
                                               -jpp
      */
      // MIPS CP0 State - First individual variables
      // Page numbers refer to revision 2.50 (July 2005) of the MIPS32 ARM, Volume III (PRA)
      unsigned CP0_IntCtl_IPTI; // Page 93, IP Timer Interrupt
      unsigned CP0_IntCtl_IPPCI; // Page 94, IP Performance Counter Interrupt
      unsigned CP0_SrsCtl_HSS; // Page 95, Highest Implemented Shadow Set
      unsigned CP0_PRId_CompanyOptions; // Page 105, Manufacture options
      unsigned CP0_PRId_CompanyID; // Page 105, Company ID - (0-255, 1=>MIPS)
      unsigned CP0_PRId_ProcessorID; // Page 105
      unsigned CP0_PRId_Revision; // Page 105
      unsigned CP0_EBase_CPUNum; // Page 106, CPU Number in a multiprocessor system
      unsigned CP0_Config_BE; // Page 108, Big/Little Endian mode
      unsigned CP0_Config_AT; //Page 109
      unsigned CP0_Config_AR; //Page 109
      unsigned CP0_Config_MT; //Page 109
      unsigned CP0_Config_VI; //Page 109
      unsigned CP0_Config1_M; // Page 110
      unsigned CP0_Config1_MMU; // Page 110
      unsigned CP0_Config1_IS; // Page 110
      unsigned CP0_Config1_IL; // Page 111
      unsigned CP0_Config1_IA; // Page 111
      unsigned CP0_Config1_DS; // Page 111
      unsigned CP0_Config1_DL; // Page 112
      unsigned CP0_Config1_DA; // Page 112
      bool CP0_Config1_C2; // Page 112
      bool CP0_Config1_MD;// Page 112 - Technically not used in MIPS32
      bool CP0_Config1_PC;// Page 112
      bool CP0_Config1_WR;// Page 113
      bool CP0_Config1_CA;// Page 113
      bool CP0_Config1_EP;// Page 113
      bool CP0_Config1_FP;// Page 113
      bool CP0_Config2_M; // Page 114
      unsigned CP0_Config2_TU;// Page 114
      unsigned CP0_Config2_TS;// Page 114
      unsigned CP0_Config2_TL;// Page 115
      unsigned CP0_Config2_TA;// Page 115
      unsigned CP0_Config2_SU;// Page 115
      unsigned CP0_Config2_SS;// Page 115
      unsigned CP0_Config2_SL;// Page 116
      unsigned CP0_Config2_SA;// Page 116
      bool CP0_Config3_M; //// Page 117
      bool CP0_Config3_DSPP;// Page 117
      bool CP0_Config3_LPA;// Page 117
      bool CP0_Config3_VEIC;// Page 118
      bool CP0_Config3_VInt; // Page 118
      bool CP0_Config3_SP;// Page 118
      bool CP0_Config3_MT;// Page 119
      bool CP0_Config3_SM;// Page 119
      bool CP0_Config3_TL;// Page 119

      bool CP0_WatchHi_M; // Page 124
      bool CP0_PerfCtr_M; // Page 130
      bool CP0_PerfCtr_W; // Page 130


      // Then, whole registers
      unsigned CP0_PRId;
      unsigned CP0_Config;
      unsigned CP0_Config1;
      unsigned CP0_Config2;
      unsigned CP0_Config3;

#endif

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
    virtual void takeOverFrom(BaseCPU *, Port *ic, Port *dc);

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

    Tick phase;

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
