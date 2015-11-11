/*
 * Copyright (c) 2012-2014 ARM Limited
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
 * Authors: Andrew Bardsley
 */

#include "arch/utility.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/fetch1.hh"
#include "cpu/minor/pipeline.hh"
#include "debug/Drain.hh"
#include "debug/MinorCPU.hh"
#include "debug/Quiesce.hh"

MinorCPU::MinorCPU(MinorCPUParams *params) :
    BaseCPU(params)
{
    /* This is only written for one thread at the moment */
    Minor::MinorThread *thread;

    if (FullSystem) {
        thread = new Minor::MinorThread(this, 0, params->system, params->itb,
            params->dtb, params->isa[0]);
    } else {
        /* thread_id 0 */
        thread = new Minor::MinorThread(this, 0, params->system,
            params->workload[0], params->itb, params->dtb, params->isa[0]);
    }

    threads.push_back(thread);

    thread->setStatus(ThreadContext::Halted);

    ThreadContext *tc = thread->getTC();

    if (params->checker) {
        fatal("The Minor model doesn't support checking (yet)\n");
    }

    threadContexts.push_back(tc);

    Minor::MinorDynInst::init();

    pipeline = new Minor::Pipeline(*this, *params);
    activityRecorder = pipeline->getActivityRecorder();
}

MinorCPU::~MinorCPU()
{
    delete pipeline;

    for (ThreadID thread_id = 0; thread_id < threads.size(); thread_id++) {
        delete threads[thread_id];
    }
}

void
MinorCPU::init()
{
    BaseCPU::init();

    if (!params()->switched_out &&
        system->getMemoryMode() != Enums::timing)
    {
        fatal("The Minor CPU requires the memory system to be in "
            "'timing' mode.\n");
    }

    /* Initialise the ThreadContext's memory proxies */
    for (ThreadID thread_id = 0; thread_id < threads.size(); thread_id++) {
        ThreadContext *tc = getContext(thread_id);

        tc->initMemProxies(tc);
    }

    /* Initialise CPUs (== threads in the ISA) */
    if (FullSystem && !params()->switched_out) {
        for (ThreadID thread_id = 0; thread_id < threads.size(); thread_id++)
        {
            ThreadContext *tc = getContext(thread_id);

            /* Initialize CPU, including PC */
            TheISA::initCPU(tc, cpuId());
        }
    }
}

/** Stats interface from SimObject (by way of BaseCPU) */
void
MinorCPU::regStats()
{
    BaseCPU::regStats();
    stats.regStats(name(), *this);
    pipeline->regStats();
}

void
MinorCPU::serializeThread(CheckpointOut &cp, ThreadID thread_id) const
{
    threads[thread_id]->serialize(cp);
}

void
MinorCPU::unserializeThread(CheckpointIn &cp, ThreadID thread_id)
{
    if (thread_id != 0)
        fatal("Trying to load more than one thread into a MinorCPU\n");

    threads[thread_id]->unserialize(cp);
}

void
MinorCPU::serialize(CheckpointOut &cp) const
{
    pipeline->serialize(cp);
    BaseCPU::serialize(cp);
}

void
MinorCPU::unserialize(CheckpointIn &cp)
{
    pipeline->unserialize(cp);
    BaseCPU::unserialize(cp);
}

Addr
MinorCPU::dbg_vtophys(Addr addr)
{
    /* Note that this gives you the translation for thread 0 */
    panic("No implementation for vtophy\n");

    return 0;
}

void
MinorCPU::wakeup(ThreadID tid)
{
    DPRINTF(Drain, "[tid:%d] MinorCPU wakeup\n", tid);

    if (threads[tid]->status() == ThreadContext::Suspended)
        threads[tid]->activate();

    DPRINTF(Drain,"Suspended Processor awoke\n");
}

void
MinorCPU::startup()
{
    DPRINTF(MinorCPU, "MinorCPU startup\n");

    BaseCPU::startup();

    for (auto i = threads.begin(); i != threads.end(); i ++)
        (*i)->startup();

    /* Workaround cases in SE mode where a thread is activated with an
     * incorrect PC that is updated after the call to activate. This
     * causes problems for Minor since it instantiates a virtual
     * branch instruction when activateContext() is called which ends
     * up pointing to an illegal address.  */
    if (threads[0]->status() == ThreadContext::Active)
        activateContext(0);
}

DrainState
MinorCPU::drain()
{
    if (switchedOut()) {
        DPRINTF(Drain, "Minor CPU switched out, draining not needed.\n");
        return DrainState::Drained;
    }

    DPRINTF(Drain, "MinorCPU drain\n");

    /* Need to suspend all threads and wait for Execute to idle.
     * Tell Fetch1 not to fetch */
    if (pipeline->drain()) {
        DPRINTF(Drain, "MinorCPU drained\n");
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "MinorCPU not finished draining\n");
        return DrainState::Draining;
    }
}

void
MinorCPU::signalDrainDone()
{
    DPRINTF(Drain, "MinorCPU drain done\n");
    Drainable::signalDrainDone();
}

void
MinorCPU::drainResume()
{
    if (switchedOut()) {
        DPRINTF(Drain, "drainResume while switched out.  Ignoring\n");
        return;
    }

    DPRINTF(Drain, "MinorCPU drainResume\n");

    if (!system->isTimingMode()) {
        fatal("The Minor CPU requires the memory system to be in "
            "'timing' mode.\n");
    }

    for (ThreadID tid = 0; tid < numThreads; tid++)
        wakeup(tid);
    pipeline->drainResume();
}

void
MinorCPU::memWriteback()
{
    DPRINTF(Drain, "MinorCPU memWriteback\n");
}

void
MinorCPU::switchOut()
{
    DPRINTF(MinorCPU, "MinorCPU switchOut\n");

    assert(!switchedOut());
    BaseCPU::switchOut();

    /* Check that the CPU is drained? */
    activityRecorder->reset();
}

void
MinorCPU::takeOverFrom(BaseCPU *old_cpu)
{
    DPRINTF(MinorCPU, "MinorCPU takeOverFrom\n");

    BaseCPU::takeOverFrom(old_cpu);

    /* Don't think I need to do anything here */
}

void
MinorCPU::activateContext(ThreadID thread_id)
{
    DPRINTF(MinorCPU, "ActivateContext thread: %d", thread_id);

    /* Do some cycle accounting.  lastStopped is reset to stop the
     *  wakeup call on the pipeline from adding the quiesce period
     *  to BaseCPU::numCycles */
    stats.quiesceCycles += pipeline->cyclesSinceLastStopped();
    pipeline->resetLastStopped();

    /* Wake up the thread, wakeup the pipeline tick */
    threads[thread_id]->activate();
    wakeupOnEvent(Minor::Pipeline::CPUStageId);
    pipeline->wakeupFetch();
}

void
MinorCPU::suspendContext(ThreadID thread_id)
{
    DPRINTF(MinorCPU, "SuspendContext %d\n", thread_id);

    threads[thread_id]->suspend();
}

void
MinorCPU::wakeupOnEvent(unsigned int stage_id)
{
    DPRINTF(Quiesce, "Event wakeup from stage %d\n", stage_id);

    /* Mark that some activity has taken place and start the pipeline */
    activityRecorder->activateStage(stage_id);
    pipeline->start();
}

MinorCPU *
MinorCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new MinorCPU(this);
}

MasterPort &MinorCPU::getInstPort()
{
    return pipeline->getInstPort();
}

MasterPort &MinorCPU::getDataPort()
{
    return pipeline->getDataPort();
}

Counter
MinorCPU::totalInsts() const
{
    Counter ret = 0;

    for (auto i = threads.begin(); i != threads.end(); i ++)
        ret += (*i)->numInst;

    return ret;
}

Counter
MinorCPU::totalOps() const
{
    Counter ret = 0;

    for (auto i = threads.begin(); i != threads.end(); i ++)
        ret += (*i)->numOp;

    return ret;
}
