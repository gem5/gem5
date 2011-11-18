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

#ifndef __CPU_OZONE_SIMPLE_PARAMS_HH__
#define __CPU_OZONE_SIMPLE_PARAMS_HH__

#include "config/the_isa.hh"
#include "cpu/ozone/cpu.hh"

//Forward declarations
namespace TheISA
{
    class TLB;
}
class FUPool;
class MemObject;
class PageTable;
class Process;
class System;

/**
 * This file defines the parameters that will be used for the OzoneCPU.
 * This must be defined externally so that the Impl can have a params class
 * defined that it can pass to all of the individual stages.
 */

class SimpleParams : public BaseCPU::Params
{
  public:

    TheISA::TLB *itb; TheISA::TLB *dtb;
    std::vector<Process *> workload;

    //Page Table
    PageTable *pTable;

    //
    // Caches
    //
//    MemInterface *icacheInterface;
//    MemInterface *dcacheInterface;

    unsigned cachePorts;
    unsigned width;
    unsigned frontEndLatency;
    unsigned frontEndWidth;
    unsigned backEndLatency;
    unsigned backEndWidth;
    unsigned backEndSquashLatency;
    unsigned maxInstBufferSize;
    unsigned numPhysicalRegs;
    unsigned maxOutstandingMemOps;
    //
    // Fetch
    //
    unsigned decodeToFetchDelay;
    unsigned renameToFetchDelay;
    unsigned iewToFetchDelay;
    unsigned commitToFetchDelay;
    unsigned fetchWidth;

    //
    // Decode
    //
    unsigned renameToDecodeDelay;
    unsigned iewToDecodeDelay;
    unsigned commitToDecodeDelay;
    unsigned fetchToDecodeDelay;
    unsigned decodeWidth;

    //
    // Rename
    //
    unsigned iewToRenameDelay;
    unsigned commitToRenameDelay;
    unsigned decodeToRenameDelay;
    unsigned renameWidth;

    //
    // IEW
    //
    unsigned commitToIEWDelay;
    unsigned renameToIEWDelay;
    unsigned issueToExecuteDelay;
    unsigned issueWidth;
    unsigned executeWidth;
    unsigned executeIntWidth;
    unsigned executeFloatWidth;
    unsigned executeBranchWidth;
    unsigned executeMemoryWidth;
    FUPool *fuPool;

    //
    // Commit
    //
    unsigned iewToCommitDelay;
    unsigned renameToROBDelay;
    unsigned commitWidth;
    unsigned squashWidth;

    //
    // Branch predictor (BP & BTB)
    //
    std::string predType;
    unsigned localPredictorSize;
    unsigned localCtrBits;
    unsigned localHistoryTableSize;
    unsigned localHistoryBits;
    unsigned globalPredictorSize;
    unsigned globalCtrBits;
    unsigned globalHistoryBits;
    unsigned choicePredictorSize;
    unsigned choiceCtrBits;

    unsigned BTBEntries;
    unsigned BTBTagSize;

    unsigned RASSize;

    //
    // Load store queue
    //
    unsigned LQEntries;
    unsigned SQEntries;
    bool lsqLimits;

    //
    // Memory dependence
    //
    unsigned SSITSize;
    unsigned LFSTSize;

    //
    // Miscellaneous
    //
    unsigned numPhysIntRegs;
    unsigned numPhysFloatRegs;
    unsigned numIQEntries;
    unsigned numROBEntries;

    bool decoupledFrontEnd;
    int dispatchWidth;
    int wbWidth;

    //SMT Parameters
    unsigned smtNumFetchingThreads;

    std::string   smtFetchPolicy;

    std::string   smtIQPolicy;
    unsigned smtIQThreshold;

    std::string   smtLSQPolicy;
    unsigned smtLSQThreshold;

    std::string   smtCommitPolicy;

    std::string   smtROBPolicy;
    unsigned smtROBThreshold;

    // Probably can get this from somewhere.
    unsigned instShiftAmt;
};

#endif // __CPU_OZONE_SIMPLE_PARAMS_HH__
