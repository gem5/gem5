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

#ifndef __BASE_CPU_HH__
#define __BASE_CPU_HH__

#include <vector>

#include "sim/eventq.hh"
#include "sim/sim_object.hh"

#include "targetarch/isa_traits.hh"	// for Addr

#ifdef FULL_SYSTEM
class System;
#endif

class BranchPred;
class ExecContext;

class BaseCPU : public SimObject
{
#ifdef FULL_SYSTEM
  protected:
    int number;
    Tick frequency;
    uint8_t interrupts[NumInterruptLevels];
    uint64_t intstatus;

  public:
    virtual void post_interrupt(int int_num, int index);
    virtual void clear_interrupt(int int_num, int index);
    virtual void clear_interrupts();

    bool check_interrupt(int int_num) const {
        if (int_num > NumInterruptLevels)
            panic("int_num out of bounds\n");

        return interrupts[int_num] != 0;
    }

    bool check_interrupts() const { return intstatus != 0; }
    uint64_t intr_status() const { return intstatus; }

    Tick getFreq() const { return frequency; }
#endif

  protected:
    std::vector<ExecContext *> contexts;

  public:
    virtual void execCtxStatusChg() {}

  public:

#ifdef FULL_SYSTEM
    BaseCPU(const std::string &_name, int _number_of_threads,
            Counter max_insts_any_thread, Counter max_insts_all_threads,
            System *_system,
            int num, Tick freq);
#else
    BaseCPU(const std::string &_name, int _number_of_threads,
            Counter max_insts_any_thread = 0,
            Counter max_insts_all_threads = 0);
#endif

    virtual ~BaseCPU() {}

    virtual void regStats();

    /// Number of threads we're actually simulating (<= SMT_MAX_THREADS).
    /// This is a constant for the duration of the simulation.
    int number_of_threads;

    /// Vector of per-thread instruction-based event queues.  Used for
    /// scheduling events based on number of instructions committed by
    /// a particular thread.
    EventQueue **comInsnEventQueue;

#ifdef FULL_SYSTEM
    System *system;
#endif

    virtual bool filterThisInstructionPrefetch(int thread_number,
        short asid, Addr prefetchTarget) const { return true; }

    /// Return pointer to CPU's branch predictor (NULL if none).
    virtual BranchPred *getBranchPred() { return NULL; };

  private:
    static std::vector<BaseCPU *> cpuList;   //!< Static global cpu list

  public:
    static int numSimulatedCPUs() { return cpuList.size(); }
};

#endif // __BASE_CPU_HH__
