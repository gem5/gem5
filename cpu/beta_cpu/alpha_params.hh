#ifndef __ALPHA_SIMPLE_PARAMS_HH__
#define __ALPHA_SIMPLE_PARAMS_HH__

#include "cpu/beta_cpu/full_cpu.hh"

//Forward declarations
class System;
class AlphaITB;
class AlphaDTB;
class FunctionalMemory;
class Process;
class MemInterface;

/**
 * This file defines the parameters that will be used for the AlphaFullCPU.
 * This must be defined externally so that the Impl can have a params class
 * defined that it can pass to all of the individual stages.
 */

class AlphaSimpleParams : public BaseFullCPU::Params
{
  public:
#ifdef FULL_SYSTEM
    AlphaITB *itb; AlphaDTB *dtb;
#else
    std::vector<Process *> workload;
    Process *process;
    short asid;
#endif // FULL_SYSTEM

    FunctionalMemory *mem;

    //
    // Caches
    //
    MemInterface *icacheInterface;
    MemInterface *dcacheInterface;

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
/*
    unsigned localPredictorSize;
    unsigned localPredictorCtrBits;
*/

    unsigned local_predictor_size;
    unsigned local_ctr_bits;
    unsigned local_history_table_size;
    unsigned local_history_bits;
    unsigned global_predictor_size;
    unsigned global_ctr_bits;
    unsigned global_history_bits;
    unsigned choice_predictor_size;
    unsigned choice_ctr_bits;

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

    // Probably can get this from somewhere.
    unsigned instShiftAmt;

    bool defReg;
};

#endif
