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
 */

#include <string>

#include "cpu/checker/cpu.hh"
#include "cpu/ozone/cpu.hh"
#include "cpu/ozone/ozone_impl.hh"
#include "cpu/ozone/simple_params.hh"
#include "cpu/inst_seq.hh"
#include "params/DerivOzoneCPU.hh"
#include "sim/process.hh"
#include "sim/sim_object.hh"

class DerivOzoneCPU : public OzoneCPU<OzoneImpl>
{
  public:
    DerivOzoneCPU(SimpleParams *p)
        : OzoneCPU<OzoneImpl>(p)
    { }
};


////////////////////////////////////////////////////////////////////////
//
//  OzoneCPU Simulation Object
//
DerivOzoneCPU *
DerivOzoneCPUParams::create()
{
    DerivOzoneCPU *cpu;

    if (FullSystem) {
        // Full-system only supports a single thread for the moment.
        ThreadID actual_num_threads = 1;
    } else {
        // In non-full-system mode, we infer the number of threads from
        // the workload if it's not explicitly specified.
        ThreadID actual_num_threads =
            numThreads.isValid() ? numThreads : workload.size();

        if (workload.size() == 0) {
            fatal("Must specify at least one workload!");
        }
    }

    SimpleParams *params = new SimpleParams;

    params->clock = clock;

    params->name = name;
    params->numberOfThreads = actual_num_threads;

    params->itb = itb;
    params->dtb = dtb;
    params->isa = isa;

    params->system = system;
    params->cpu_id = cpu_id;
    params->profile = profile;
    params->do_quiesce = do_quiesce;
    params->do_checkpoint_insts = do_checkpoint_insts;
    params->do_statistics_insts = do_statistics_insts;
    params->workload = workload;

    params->checker = checker;
    params->max_insts_any_thread = max_insts_any_thread;
    params->max_insts_all_threads = max_insts_all_threads;
    params->max_loads_any_thread = max_loads_any_thread;
    params->max_loads_all_threads = max_loads_all_threads;
    params->progress_interval = progress_interval;

    //
    // Caches
    //
//    params->icacheInterface = icache ? icache->getInterface() : NULL;
//    params->dcacheInterface = dcache ? dcache->getInterface() : NULL;
    params->cachePorts = cachePorts;

    params->width = width;
    params->frontEndWidth = frontEndWidth;
    params->frontEndLatency = frontEndLatency;
    params->backEndWidth = backEndWidth;
    params->backEndSquashLatency = backEndSquashLatency;
    params->backEndLatency = backEndLatency;
    params->maxInstBufferSize = maxInstBufferSize;
    params->numPhysicalRegs = numPhysIntRegs + numPhysFloatRegs;
    params->maxOutstandingMemOps = maxOutstandingMemOps;

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
    params->issueWidth = issueWidth;
    params->executeWidth = executeWidth;
    params->executeIntWidth = executeIntWidth;
    params->executeFloatWidth = executeFloatWidth;
    params->executeBranchWidth = executeBranchWidth;
    params->executeMemoryWidth = executeMemoryWidth;

    params->iewToCommitDelay = iewToCommitDelay;
    params->renameToROBDelay = renameToROBDelay;
    params->commitWidth = commitWidth;
    params->squashWidth = squashWidth;

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
    params->lsqLimits = lsqLimits;

    params->SSITSize = SSITSize;
    params->LFSTSize = LFSTSize;

    params->numPhysIntRegs = numPhysIntRegs;
    params->numPhysFloatRegs = numPhysFloatRegs;
    params->numIQEntries = numIQEntries;
    params->numROBEntries = numROBEntries;

    params->decoupledFrontEnd = decoupledFrontEnd;
    params->dispatchWidth = dispatchWidth;
    params->wbWidth = wbWidth;

    params->smtNumFetchingThreads = smtNumFetchingThreads;
    params->smtFetchPolicy = smtFetchPolicy;
    params->smtIQPolicy    = smtIQPolicy;
    params->smtLSQPolicy    = smtLSQPolicy;
    params->smtLSQThreshold = smtLSQThreshold;
    params->smtROBPolicy   = smtROBPolicy;
    params->smtROBThreshold = smtROBThreshold;
    params->smtCommitPolicy = smtCommitPolicy;

    params->instShiftAmt = 2;

    params->switched_out = switched_out;

    params->functionTrace = function_trace;
    params->functionTraceStart = function_trace_start;

    cpu = new DerivOzoneCPU(params);

    return cpu;
}
