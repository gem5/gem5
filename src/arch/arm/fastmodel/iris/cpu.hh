/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_IRIS_CPU_HH__
#define __ARCH_ARM_FASTMODEL_IRIS_CPU_HH__

#include "cpu/base.hh"
#include "iris/detail/IrisInterface.h"
#include "params/IrisBaseCPU.hh"
#include "systemc/ext/core/sc_attr.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"

namespace gem5
{

namespace Iris
{

class ThreadContext;

// The base interface of the EVS used by gem5 BaseCPU below.
class BaseCpuEvs
{
  public:
    virtual void setClkPeriod(Tick clk_period) = 0;
    virtual void setSysCounterFrq(uint64_t sys_counter_frq) = 0;
    virtual void setCluster(SimObject *cluster) = 0;
    virtual void setResetAddr(int core, Addr addr, bool secure) = 0;
};

// This CPU class adds some mechanisms which help attach the gem5 and fast
// model CPUs to each other. It acts as a base class for the gem5 CPU, and
// holds a pointer to the EVS. It also has some methods for setting up some
// attributes in the fast model CPU to control its clock rate.
class BaseCPU : public gem5::BaseCPU
{
  public:
    BaseCPU(const BaseCPUParams &params, sc_core::sc_module *_evs);
    virtual ~BaseCPU();

    Port &
    getDataPort() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    Port &
    getInstPort() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    wakeup(ThreadID tid) override
    {
        auto *tc = threadContexts.at(tid);
        if (tc->status() == gem5::ThreadContext::Suspended)
            tc->activate();
    }

    Counter totalInsts() const override;

    Counter
    totalOps() const override
    {
        return totalInsts();
    }

    virtual void
    setResetAddr(Addr addr, bool secure = false)
    {
        panic("%s not implemented.", __FUNCTION__);
    }

  protected:
    sc_core::sc_module *evs;
    // Hold casted pointer to *evs.
    Iris::BaseCpuEvs *evs_base_cpu;

  protected:
    friend ThreadContext;

    void
    clockPeriodUpdated() override
    {
        evs_base_cpu->setClkPeriod(clockPeriod());
    }

    void serializeThread(CheckpointOut &cp, ThreadID tid) const override;
};

// This class specializes the one above and sets up ThreadContexts based on
// its template parameters. These ThreadContexts provide the standard gem5
// interface and translate those accesses to use the Iris API to access that
// state in the target context.
template <class TC>
class CPU : public Iris::BaseCPU
{
  public:
    CPU(const IrisBaseCPUParams &params,
        iris::IrisConnectionInterface *iris_if)
        : BaseCPU(params, params.evs)
    {
        const std::string parent_path = evs->name();
        System *sys = params.system;

        int thread_id = 0;
        for (const std::string &sub_path : params.thread_paths) {
            std::string path = parent_path + "." + sub_path;
            auto id = thread_id++;
            auto *tc = new TC(this, id, sys, params.mmu, params.isa[id],
                              iris_if, path);
            threadContexts.push_back(tc);
        }
    }
};

} // namespace Iris
} // namespace gem5

#endif // __ARCH_ARM_FASTMODEL_IRIS_CPU_HH__
