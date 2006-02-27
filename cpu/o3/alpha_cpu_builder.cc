/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "base/inifile.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/o3/alpha_cpu.hh"
#include "cpu/o3/alpha_impl.hh"
#include "mem/base_mem.hh"
#include "mem/cache/base_cache.hh"
#include "mem/mem_interface.hh"
#include "sim/builder.hh"
#include "sim/debug.hh"
#include "sim/host.hh"
#include "sim/process.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

#if FULL_SYSTEM
#include "base/remote_gdb.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/system.hh"
#include "arch/tlb.hh"
#include "arch/vtophys.hh"
#else // !FULL_SYSTEM
#include "mem/functional/functional.hh"
#endif // FULL_SYSTEM

class DerivAlphaFullCPU : public AlphaFullCPU<AlphaSimpleImpl>
{
  public:
    DerivAlphaFullCPU(AlphaSimpleParams p)
        : AlphaFullCPU<AlphaSimpleImpl>(p)
    { }
};

BEGIN_DECLARE_SIM_OBJECT_PARAMS(DerivAlphaFullCPU)

    Param<int> clock;
    Param<int> numThreads;

#if FULL_SYSTEM
SimObjectParam<System *> system;
Param<int> cpu_id;
SimObjectParam<AlphaITB *> itb;
SimObjectParam<AlphaDTB *> dtb;
#else
SimObjectVectorParam<Process *> workload;
#endif // FULL_SYSTEM
SimObjectParam<FunctionalMemory *> mem;

Param<Counter> max_insts_any_thread;
Param<Counter> max_insts_all_threads;
Param<Counter> max_loads_any_thread;
Param<Counter> max_loads_all_threads;

SimObjectParam<BaseCache *> icache;
SimObjectParam<BaseCache *> dcache;

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
Param<unsigned> issueWidth;
Param<unsigned> executeWidth;
Param<unsigned> executeIntWidth;
Param<unsigned> executeFloatWidth;
Param<unsigned> executeBranchWidth;
Param<unsigned> executeMemoryWidth;

Param<unsigned> iewToCommitDelay;
Param<unsigned> renameToROBDelay;
Param<unsigned> commitWidth;
Param<unsigned> squashWidth;

#if 0
Param<unsigned> localPredictorSize;
Param<unsigned> localPredictorCtrBits;
#endif
Param<unsigned> local_predictor_size;
Param<unsigned> local_ctr_bits;
Param<unsigned> local_history_table_size;
Param<unsigned> local_history_bits;
Param<unsigned> global_predictor_size;
Param<unsigned> global_ctr_bits;
Param<unsigned> global_history_bits;
Param<unsigned> choice_predictor_size;
Param<unsigned> choice_ctr_bits;

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

Param<unsigned> instShiftAmt;

Param<bool> defer_registration;

Param<bool> function_trace;
Param<Tick> function_trace_start;

END_DECLARE_SIM_OBJECT_PARAMS(DerivAlphaFullCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(DerivAlphaFullCPU)

    INIT_PARAM(clock, "clock speed"),
    INIT_PARAM(numThreads, "number of HW thread contexts"),

#if FULL_SYSTEM
    INIT_PARAM(system, "System object"),
    INIT_PARAM(cpu_id, "processor ID"),
    INIT_PARAM(itb, "Instruction translation buffer"),
    INIT_PARAM(dtb, "Data translation buffer"),
#else
    INIT_PARAM(workload, "Processes to run"),
#endif // FULL_SYSTEM

    INIT_PARAM_DFLT(mem, "Memory", NULL),

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

    INIT_PARAM_DFLT(icache, "L1 instruction cache", NULL),
    INIT_PARAM_DFLT(dcache, "L1 data cache", NULL),

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
    INIT_PARAM(issueWidth, "Issue width"),
    INIT_PARAM(executeWidth, "Execute width"),
    INIT_PARAM(executeIntWidth, "Integer execute width"),
    INIT_PARAM(executeFloatWidth, "Floating point execute width"),
    INIT_PARAM(executeBranchWidth, "Branch execute width"),
    INIT_PARAM(executeMemoryWidth, "Memory execute width"),

    INIT_PARAM(iewToCommitDelay, "Issue/Execute/Writeback to commit "
               "delay"),
    INIT_PARAM(renameToROBDelay, "Rename to reorder buffer delay"),
    INIT_PARAM(commitWidth, "Commit width"),
    INIT_PARAM(squashWidth, "Squash width"),

#if 0
    INIT_PARAM(localPredictorSize, "Size of the local predictor in entries. "
               "Must be a power of 2."),
    INIT_PARAM(localPredictorCtrBits, "Number of bits per counter for bpred"),
#endif
    INIT_PARAM(local_predictor_size, "Size of local predictor"),
    INIT_PARAM(local_ctr_bits, "Bits per counter"),
    INIT_PARAM(local_history_table_size, "Size of local history table"),
    INIT_PARAM(local_history_bits, "Bits for the local history"),
    INIT_PARAM(global_predictor_size, "Size of global predictor"),
    INIT_PARAM(global_ctr_bits, "Bits per counter"),
    INIT_PARAM(global_history_bits, "Bits of history"),
    INIT_PARAM(choice_predictor_size, "Size of choice predictor"),
    INIT_PARAM(choice_ctr_bits, "Bits of choice counters"),

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

    INIT_PARAM(instShiftAmt, "Number of bits to shift instructions by"),
    INIT_PARAM(defer_registration, "defer system registration (for sampling)"),

    INIT_PARAM(function_trace, "Enable function trace"),
    INIT_PARAM(function_trace_start, "Cycle to start function trace")

END_INIT_SIM_OBJECT_PARAMS(DerivAlphaFullCPU)

CREATE_SIM_OBJECT(DerivAlphaFullCPU)
{
    DerivAlphaFullCPU *cpu;

#if FULL_SYSTEM
    // Full-system only supports a single thread for the moment.
    int actual_num_threads = 1;
#else
    // In non-full-system mode, we infer the number of threads from
    // the workload if it's not explicitly specified.
    int actual_num_threads =
        numThreads.isValid() ? numThreads : workload.size();

    if (workload.size() == 0) {
        fatal("Must specify at least one workload!");
    }

#endif

    AlphaSimpleParams params;

    params.clock = clock;

    params.name = getInstanceName();
    params.numberOfThreads = actual_num_threads;

#if FULL_SYSTEM
    params.system = system;
    params.cpu_id = cpu_id;
    params.itb = itb;
    params.dtb = dtb;
#else
    params.workload = workload;
#endif // FULL_SYSTEM

    params.mem = mem;

    params.max_insts_any_thread = max_insts_any_thread;
    params.max_insts_all_threads = max_insts_all_threads;
    params.max_loads_any_thread = max_loads_any_thread;
    params.max_loads_all_threads = max_loads_all_threads;

    //
    // Caches
    //
    params.icacheInterface = icache ? icache->getInterface() : NULL;
    params.dcacheInterface = dcache ? dcache->getInterface() : NULL;

    params.decodeToFetchDelay = decodeToFetchDelay;
    params.renameToFetchDelay = renameToFetchDelay;
    params.iewToFetchDelay = iewToFetchDelay;
    params.commitToFetchDelay = commitToFetchDelay;
    params.fetchWidth = fetchWidth;

    params.renameToDecodeDelay = renameToDecodeDelay;
    params.iewToDecodeDelay = iewToDecodeDelay;
    params.commitToDecodeDelay = commitToDecodeDelay;
    params.fetchToDecodeDelay = fetchToDecodeDelay;
    params.decodeWidth = decodeWidth;

    params.iewToRenameDelay = iewToRenameDelay;
    params.commitToRenameDelay = commitToRenameDelay;
    params.decodeToRenameDelay = decodeToRenameDelay;
    params.renameWidth = renameWidth;

    params.commitToIEWDelay = commitToIEWDelay;
    params.renameToIEWDelay = renameToIEWDelay;
    params.issueToExecuteDelay = issueToExecuteDelay;
    params.issueWidth = issueWidth;
    params.executeWidth = executeWidth;
    params.executeIntWidth = executeIntWidth;
    params.executeFloatWidth = executeFloatWidth;
    params.executeBranchWidth = executeBranchWidth;
    params.executeMemoryWidth = executeMemoryWidth;

    params.iewToCommitDelay = iewToCommitDelay;
    params.renameToROBDelay = renameToROBDelay;
    params.commitWidth = commitWidth;
    params.squashWidth = squashWidth;
#if 0
    params.localPredictorSize = localPredictorSize;
    params.localPredictorCtrBits = localPredictorCtrBits;
#endif
    params.local_predictor_size = local_predictor_size;
    params.local_ctr_bits = local_ctr_bits;
    params.local_history_table_size = local_history_table_size;
    params.local_history_bits = local_history_bits;
    params.global_predictor_size = global_predictor_size;
    params.global_ctr_bits = global_ctr_bits;
    params.global_history_bits = global_history_bits;
    params.choice_predictor_size = choice_predictor_size;
    params.choice_ctr_bits = choice_ctr_bits;

    params.BTBEntries = BTBEntries;
    params.BTBTagSize = BTBTagSize;

    params.RASSize = RASSize;

    params.LQEntries = LQEntries;
    params.SQEntries = SQEntries;
    params.SSITSize = SSITSize;
    params.LFSTSize = LFSTSize;

    params.numPhysIntRegs = numPhysIntRegs;
    params.numPhysFloatRegs = numPhysFloatRegs;
    params.numIQEntries = numIQEntries;
    params.numROBEntries = numROBEntries;

    params.instShiftAmt = 2;

    params.defReg = defer_registration;

    params.functionTrace = function_trace;
    params.functionTraceStart = function_trace_start;

    cpu = new DerivAlphaFullCPU(params);

    return cpu;
}

REGISTER_SIM_OBJECT("DerivAlphaFullCPU", DerivAlphaFullCPU)

