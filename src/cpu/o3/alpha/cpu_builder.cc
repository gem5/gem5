/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 */

#include <string>

#include "cpu/base.hh"
#include "cpu/o3/alpha/cpu.hh"
#include "cpu/o3/alpha/impl.hh"
#include "cpu/o3/alpha/params.hh"
#include "cpu/o3/fu_pool.hh"
#include "sim/builder.hh"

class DerivO3CPU : public AlphaO3CPU<AlphaSimpleImpl>
{
  public:
    DerivO3CPU(AlphaSimpleParams *p)
        : AlphaO3CPU<AlphaSimpleImpl>(p)
    { }
};

BEGIN_DECLARE_SIM_OBJECT_PARAMS(DerivO3CPU)

    Param<int> clock;
    Param<int> phase;
    Param<int> numThreads;
Param<int> cpu_id;
Param<int> activity;

#if FULL_SYSTEM
SimObjectParam<System *> system;
SimObjectParam<AlphaISA::ITB *> itb;
SimObjectParam<AlphaISA::DTB *> dtb;
Param<Tick> profile;

Param<bool> do_quiesce;
Param<bool> do_checkpoint_insts;
Param<bool> do_statistics_insts;
#else
SimObjectVectorParam<Process *> workload;
#endif // FULL_SYSTEM

SimObjectParam<BaseCPU *> checker;

Param<Counter> max_insts_any_thread;
Param<Counter> max_insts_all_threads;
Param<Counter> max_loads_any_thread;
Param<Counter> max_loads_all_threads;
Param<Tick> progress_interval;

Param<unsigned> cachePorts;

Param<unsigned> decodeToFetchDelay;
Param<unsigned> renameToFetchDelay;
Param<unsigned> iewToFetchDelay;
Param<unsigned> commitToFetchDelay;
Param<unsigned> fetchWidth;

Param<unsigned> renameToDecodeDelay;
Param<unsigned> iewToDecodeDelay;
Param<unsigned> commitToDecodeDelay;
Param<unsigned> fetchToDecodeDelay;
Param<unsigned> decodeWidth;

Param<unsigned> iewToRenameDelay;
Param<unsigned> commitToRenameDelay;
Param<unsigned> decodeToRenameDelay;
Param<unsigned> renameWidth;

Param<unsigned> commitToIEWDelay;
Param<unsigned> renameToIEWDelay;
Param<unsigned> issueToExecuteDelay;
Param<unsigned> dispatchWidth;
Param<unsigned> issueWidth;
Param<unsigned> wbWidth;
Param<unsigned> wbDepth;
SimObjectParam<FUPool *> fuPool;

Param<unsigned> iewToCommitDelay;
Param<unsigned> renameToROBDelay;
Param<unsigned> commitWidth;
Param<unsigned> squashWidth;
Param<Tick> trapLatency;

Param<unsigned> backComSize;
Param<unsigned> forwardComSize;

Param<std::string> predType;
Param<unsigned> localPredictorSize;
Param<unsigned> localCtrBits;
Param<unsigned> localHistoryTableSize;
Param<unsigned> localHistoryBits;
Param<unsigned> globalPredictorSize;
Param<unsigned> globalCtrBits;
Param<unsigned> globalHistoryBits;
Param<unsigned> choicePredictorSize;
Param<unsigned> choiceCtrBits;

Param<unsigned> BTBEntries;
Param<unsigned> BTBTagSize;

Param<unsigned> RASSize;

Param<unsigned> LQEntries;
Param<unsigned> SQEntries;
Param<unsigned> LFSTSize;
Param<unsigned> SSITSize;

Param<unsigned> numPhysIntRegs;
Param<unsigned> numPhysFloatRegs;
Param<unsigned> numIQEntries;
Param<unsigned> numROBEntries;

Param<unsigned> smtNumFetchingThreads;
Param<std::string>   smtFetchPolicy;
Param<std::string>   smtLSQPolicy;
Param<unsigned> smtLSQThreshold;
Param<std::string>   smtIQPolicy;
Param<unsigned> smtIQThreshold;
Param<std::string>   smtROBPolicy;
Param<unsigned> smtROBThreshold;
Param<std::string>   smtCommitPolicy;

Param<unsigned> instShiftAmt;

Param<bool> defer_registration;

Param<bool> function_trace;
Param<Tick> function_trace_start;

END_DECLARE_SIM_OBJECT_PARAMS(DerivO3CPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(DerivO3CPU)

    INIT_PARAM(clock, "clock speed"),
    INIT_PARAM_DFLT(phase, "clock phase", 0),
    INIT_PARAM(numThreads, "number of HW thread contexts"),
    INIT_PARAM(cpu_id, "processor ID"),
    INIT_PARAM_DFLT(activity, "Initial activity count", 0),

#if FULL_SYSTEM
    INIT_PARAM(system, "System object"),
    INIT_PARAM(itb, "Instruction translation buffer"),
    INIT_PARAM(dtb, "Data translation buffer"),
    INIT_PARAM(profile, ""),

    INIT_PARAM(do_quiesce, ""),
    INIT_PARAM(do_checkpoint_insts, ""),
    INIT_PARAM(do_statistics_insts, ""),
#else
    INIT_PARAM(workload, "Processes to run"),
#endif // FULL_SYSTEM

    INIT_PARAM_DFLT(checker, "Checker CPU", NULL),

    INIT_PARAM_DFLT(max_insts_any_thread,
                    "Terminate when any thread reaches this inst count",
                    0),
    INIT_PARAM_DFLT(max_insts_all_threads,
                    "Terminate when all threads have reached"
                    "this inst count",
                    0),
    INIT_PARAM_DFLT(max_loads_any_thread,
                    "Terminate when any thread reaches this load count",
                    0),
    INIT_PARAM_DFLT(max_loads_all_threads,
                    "Terminate when all threads have reached this load"
                    "count",
                    0),
    INIT_PARAM_DFLT(progress_interval, "Progress interval", 0),

    INIT_PARAM_DFLT(cachePorts, "Cache Ports", 200),

    INIT_PARAM(decodeToFetchDelay, "Decode to fetch delay"),
    INIT_PARAM(renameToFetchDelay, "Rename to fetch delay"),
    INIT_PARAM(iewToFetchDelay, "Issue/Execute/Writeback to fetch"
               "delay"),
    INIT_PARAM(commitToFetchDelay, "Commit to fetch delay"),
    INIT_PARAM(fetchWidth, "Fetch width"),
    INIT_PARAM(renameToDecodeDelay, "Rename to decode delay"),
    INIT_PARAM(iewToDecodeDelay, "Issue/Execute/Writeback to decode"
               "delay"),
    INIT_PARAM(commitToDecodeDelay, "Commit to decode delay"),
    INIT_PARAM(fetchToDecodeDelay, "Fetch to decode delay"),
    INIT_PARAM(decodeWidth, "Decode width"),

    INIT_PARAM(iewToRenameDelay, "Issue/Execute/Writeback to rename"
               "delay"),
    INIT_PARAM(commitToRenameDelay, "Commit to rename delay"),
    INIT_PARAM(decodeToRenameDelay, "Decode to rename delay"),
    INIT_PARAM(renameWidth, "Rename width"),

    INIT_PARAM(commitToIEWDelay, "Commit to "
               "Issue/Execute/Writeback delay"),
    INIT_PARAM(renameToIEWDelay, "Rename to "
               "Issue/Execute/Writeback delay"),
    INIT_PARAM(issueToExecuteDelay, "Issue to execute delay (internal"
               "to the IEW stage)"),
    INIT_PARAM(dispatchWidth, "Dispatch width"),
    INIT_PARAM(issueWidth, "Issue width"),
    INIT_PARAM(wbWidth, "Writeback width"),
    INIT_PARAM(wbDepth, "Writeback depth (number of cycles it can buffer)"),
    INIT_PARAM_DFLT(fuPool, "Functional unit pool", NULL),

    INIT_PARAM(iewToCommitDelay, "Issue/Execute/Writeback to commit "
               "delay"),
    INIT_PARAM(renameToROBDelay, "Rename to reorder buffer delay"),
    INIT_PARAM(commitWidth, "Commit width"),
    INIT_PARAM(squashWidth, "Squash width"),
    INIT_PARAM_DFLT(trapLatency, "Number of cycles before the trap is handled", 6),

    INIT_PARAM(backComSize, "Time buffer size for backwards communication"),
    INIT_PARAM(forwardComSize, "Time buffer size for forward communication"),

    INIT_PARAM(predType, "Type of branch predictor ('local', 'tournament')"),
    INIT_PARAM(localPredictorSize, "Size of local predictor"),
    INIT_PARAM(localCtrBits, "Bits per counter"),
    INIT_PARAM(localHistoryTableSize, "Size of local history table"),
    INIT_PARAM(localHistoryBits, "Bits for the local history"),
    INIT_PARAM(globalPredictorSize, "Size of global predictor"),
    INIT_PARAM(globalCtrBits, "Bits per counter"),
    INIT_PARAM(globalHistoryBits, "Bits of history"),
    INIT_PARAM(choicePredictorSize, "Size of choice predictor"),
    INIT_PARAM(choiceCtrBits, "Bits of choice counters"),

    INIT_PARAM(BTBEntries, "Number of BTB entries"),
    INIT_PARAM(BTBTagSize, "Size of the BTB tags, in bits"),

    INIT_PARAM(RASSize, "RAS size"),

    INIT_PARAM(LQEntries, "Number of load queue entries"),
    INIT_PARAM(SQEntries, "Number of store queue entries"),
    INIT_PARAM(LFSTSize, "Last fetched store table size"),
    INIT_PARAM(SSITSize, "Store set ID table size"),

    INIT_PARAM(numPhysIntRegs, "Number of physical integer registers"),
    INIT_PARAM(numPhysFloatRegs, "Number of physical floating point "
               "registers"),
    INIT_PARAM(numIQEntries, "Number of instruction queue entries"),
    INIT_PARAM(numROBEntries, "Number of reorder buffer entries"),

    INIT_PARAM_DFLT(smtNumFetchingThreads, "SMT Number of Fetching Threads", 1),
    INIT_PARAM_DFLT(smtFetchPolicy, "SMT Fetch Policy", "SingleThread"),
    INIT_PARAM_DFLT(smtLSQPolicy,   "SMT LSQ Sharing Policy",    "Partitioned"),
    INIT_PARAM_DFLT(smtLSQThreshold,"SMT LSQ Threshold", 100),
    INIT_PARAM_DFLT(smtIQPolicy,    "SMT IQ Policy",    "Partitioned"),
    INIT_PARAM_DFLT(smtIQThreshold, "SMT IQ Threshold", 100),
    INIT_PARAM_DFLT(smtROBPolicy,   "SMT ROB Sharing Policy", "Partitioned"),
    INIT_PARAM_DFLT(smtROBThreshold,"SMT ROB Threshold", 100),
    INIT_PARAM_DFLT(smtCommitPolicy,"SMT Commit Fetch Policy", "RoundRobin"),

    INIT_PARAM(instShiftAmt, "Number of bits to shift instructions by"),
    INIT_PARAM(defer_registration, "defer system registration (for sampling)"),

    INIT_PARAM(function_trace, "Enable function trace"),
    INIT_PARAM(function_trace_start, "Cycle to start function trace")

END_INIT_SIM_OBJECT_PARAMS(DerivO3CPU)

CREATE_SIM_OBJECT(DerivO3CPU)
{
    DerivO3CPU *cpu;

#if FULL_SYSTEM
    // Full-system only supports a single thread for the moment.
    int actual_num_threads = 1;
#else
    // In non-full-system mode, we infer the number of threads from
    // the workload if it's not explicitly specified.
    int actual_num_threads =
        (numThreads.isValid() && numThreads >= workload.size()) ?
         numThreads : workload.size();

    if (workload.size() == 0) {
        fatal("Must specify at least one workload!");
    }
#endif

    AlphaSimpleParams *params = new AlphaSimpleParams;

    params->clock = clock;
    params->phase = phase;

    params->name = getInstanceName();
    params->numberOfThreads = actual_num_threads;
    params->cpu_id = cpu_id;
    params->activity = activity;

#if FULL_SYSTEM
    params->system = system;
    params->itb = itb;
    params->dtb = dtb;
    params->profile = profile;

    params->do_quiesce = do_quiesce;
    params->do_checkpoint_insts = do_checkpoint_insts;
    params->do_statistics_insts = do_statistics_insts;
#else
    params->workload = workload;
#endif // FULL_SYSTEM

    params->checker = checker;

    params->max_insts_any_thread = max_insts_any_thread;
    params->max_insts_all_threads = max_insts_all_threads;
    params->max_loads_any_thread = max_loads_any_thread;
    params->max_loads_all_threads = max_loads_all_threads;
    params->progress_interval = progress_interval;

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

REGISTER_SIM_OBJECT("DerivO3CPU", DerivO3CPU)

