/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#include <string>

#include "config/use_checker.hh"
#include "cpu/base.hh"
#include "cpu/o3/mips/cpu.hh"
#include "cpu/o3/mips/impl.hh"
#include "cpu/o3/mips/params.hh"
#include "cpu/o3/fu_pool.hh"
#include "params/DerivO3CPU.hh"

class DerivO3CPU : public MipsO3CPU<MipsSimpleImpl>
{
  public:
    DerivO3CPU(MipsSimpleParams *p)
        : MipsO3CPU<MipsSimpleImpl>(p)
    { }
};

DerivO3CPU *
DerivO3CPUParams::create()
{
    DerivO3CPU *cpu;

    // In non-full-system mode, we infer the number of threads from
    // the workload if it's not explicitly specified.
    int actual_num_threads =
        (numThreads >= workload.size()) ? numThreads : workload.size();

    if (workload.size() == 0) {
        fatal("Must specify at least one workload!");
    }

    MipsSimpleParams *params = new MipsSimpleParams;

    params->clock = clock;
    params->phase = phase;

    params->name = name;
    params->numberOfThreads = actual_num_threads;
    params->cpu_id = cpu_id;
    params->activity = activity;

    params->workload = workload;

#if USE_CHECKER
    params->checker = checker;
#endif

    params->max_insts_any_thread = max_insts_any_thread;
    params->max_insts_all_threads = max_insts_all_threads;
    params->max_loads_any_thread = max_loads_any_thread;
    params->max_loads_all_threads = max_loads_all_threads;

    //
    // Caches
    //
    params->cachePorts = cachePorts;

    params->decodeToFetchDelay = decodeToFetchDelay;
    params->renameToFetchDelay = renameToFetchDelay;
    params->iewToFetchDelay = iewToFetchDelay;
    params->commitToFetchDelay = commitToFetchDelay;
    params->fetchWidth = fetchWidth;

    params->renameToDecodeDelay = renameToDecodeDelay;
    params->iewToDecodeDelay = iewToDecodeDelay;
    params->commitToDecodeDelay = commitToDecodeDelay;
    params->fetchToDecodeDelay = fetchToDecodeDelay;
    params->decodeWidth = decodeWidth;

    params->iewToRenameDelay = iewToRenameDelay;
    params->commitToRenameDelay = commitToRenameDelay;
    params->decodeToRenameDelay = decodeToRenameDelay;
    params->renameWidth = renameWidth;

    params->commitToIEWDelay = commitToIEWDelay;
    params->renameToIEWDelay = renameToIEWDelay;
    params->issueToExecuteDelay = issueToExecuteDelay;
    params->dispatchWidth = dispatchWidth;
    params->issueWidth = issueWidth;
    params->wbWidth = wbWidth;
    params->wbDepth = wbDepth;
    params->fuPool = fuPool;

    params->iewToCommitDelay = iewToCommitDelay;
    params->renameToROBDelay = renameToROBDelay;
    params->commitWidth = commitWidth;
    params->squashWidth = squashWidth;
    params->trapLatency = trapLatency;

    params->backComSize = backComSize;
    params->forwardComSize = forwardComSize;

    params->predType = predType;
    params->localPredictorSize = localPredictorSize;
    params->localCtrBits = localCtrBits;
    params->localHistoryTableSize = localHistoryTableSize;
    params->localHistoryBits = localHistoryBits;
    params->globalPredictorSize = globalPredictorSize;
    params->globalCtrBits = globalCtrBits;
    params->globalHistoryBits = globalHistoryBits;
    params->choicePredictorSize = choicePredictorSize;
    params->choiceCtrBits = choiceCtrBits;

    params->BTBEntries = BTBEntries;
    params->BTBTagSize = BTBTagSize;

    params->RASSize = RASSize;

    params->LQEntries = LQEntries;
    params->SQEntries = SQEntries;

    params->SSITSize = SSITSize;
    params->LFSTSize = LFSTSize;

    params->numPhysIntRegs = numPhysIntRegs;
    params->numPhysFloatRegs = numPhysFloatRegs;
    params->numIQEntries = numIQEntries;
    params->numROBEntries = numROBEntries;

    params->smtNumFetchingThreads = smtNumFetchingThreads;

    // Default smtFetchPolicy to "RoundRobin", if necessary.
    std::string round_robin_policy = "RoundRobin";
    std::string single_thread = "SingleThread";

    if (actual_num_threads > 1 && single_thread.compare(smtFetchPolicy) == 0)
        params->smtFetchPolicy = round_robin_policy;
    else
        params->smtFetchPolicy = smtFetchPolicy;

    params->smtIQPolicy    = smtIQPolicy;
    params->smtLSQPolicy    = smtLSQPolicy;
    params->smtLSQThreshold = smtLSQThreshold;
    params->smtROBPolicy   = smtROBPolicy;
    params->smtROBThreshold = smtROBThreshold;
    params->smtCommitPolicy = smtCommitPolicy;

    params->instShiftAmt = 2;

    params->deferRegistration = defer_registration;

    params->functionTrace = function_trace;
    params->functionTraceStart = function_trace_start;

    cpu = new DerivO3CPU(params);

    return cpu;
}
