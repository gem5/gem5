#ifndef __ALPHA_SIMPLE_PARAMS_HH__
#define __ALPHA_SIMPLE_PARAMS_HH__

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

class AlphaSimpleParams
{
  public:
    std::string name;
    int numberOfThreads;

#ifdef FULL_SYSTEM
    System *_system;
    AlphaITB *itb; AlphaDTB *dtb;
    Tick freq;
#else
    std::vector<Process *> workload;
    Process *process;
    short asid;
#endif // FULL_SYSTEM

    FunctionalMemory *mem;

    Counter maxInstsAnyThread;
    Counter maxInstsAllThreads;
    Counter maxLoadsAnyThread;
    Counter maxLoadsAllThreads;

    //
    // Caches
    //
    MemInterface *icacheInterface;
    MemInterface *dcacheInterface;

    unsigned decodeToFetchDelay;
    unsigned renameToFetchDelay;
    unsigned iewToFetchDelay;
    unsigned commitToFetchDelay;
    unsigned fetchWidth;

    unsigned renameToDecodeDelay;
    unsigned iewToDecodeDelay;
    unsigned commitToDecodeDelay;
    unsigned fetchToDecodeDelay;
    unsigned decodeWidth;

    unsigned iewToRenameDelay;
    unsigned commitToRenameDelay;
    unsigned decodeToRenameDelay;
    unsigned renameWidth;

    unsigned commitToIEWDelay;
    unsigned renameToIEWDelay;
    unsigned issueToExecuteDelay;
    unsigned issueWidth;
    unsigned executeWidth;
    unsigned executeIntWidth;
    unsigned executeFloatWidth;

    unsigned iewToCommitDelay;
    unsigned renameToROBDelay;
    unsigned commitWidth;
    unsigned squashWidth;

    unsigned numPhysIntRegs;
    unsigned numPhysFloatRegs;
    unsigned numIQEntries;
    unsigned numROBEntries;

    bool defReg;
};

#endif
