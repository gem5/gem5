

#ifndef __CPU_OZONE_SIMPLE_PARAMS_HH__
#define __CPU_OZONE_SIMPLE_PARAMS_HH__

#include "cpu/ozone/cpu.hh"

//Forward declarations
class AlphaDTB;
class AlphaITB;
class FUPool;
class FunctionalMemory;
class MemInterface;
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

#if FULL_SYSTEM
    AlphaITB *itb; AlphaDTB *dtb;
#else
    std::vector<Process *> workload;
//    Process *process;
#endif // FULL_SYSTEM

    //Page Table
    PageTable *pTable;

    FunctionalMemory *mem;

    //
    // Caches
    //
    MemInterface *icacheInterface;
    MemInterface *dcacheInterface;

    unsigned cachePorts;
    unsigned width;
    unsigned frontEndWidth;
    unsigned backEndWidth;
    unsigned backEndSquashLatency;
    unsigned backEndLatency;
    unsigned maxInstBufferSize;
    unsigned numPhysicalRegs;
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
