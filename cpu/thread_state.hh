
#ifndef __CPU_THREAD_STATE_HH__
#define __CPU_THREAD_STATE_HH__

#include "cpu/exec_context.hh"

#if FULL_SYSTEM
class EndQuiesceEvent;
class FunctionProfile;
class ProfileNode;
#else
class Process;
class FunctionalMemory;
#endif

struct ThreadState {
#if FULL_SYSTEM
    ThreadState(int _cpuId, int _tid, FunctionalMemory *_mem)
        : cpuId(_cpuId), tid(_tid), mem(_mem), lastActivate(0), lastSuspend(0),
          profile(NULL), profileNode(NULL), profilePC(0), quiesceEvent(NULL)
#else
    ThreadState(int _cpuId, int _tid, FunctionalMemory *_mem,
                Process *_process, short _asid)
        : cpuId(_cpuId), tid(_tid), mem(_mem), process(_process), asid(_asid)
#endif
    {
        funcExeInst = 0;
        storeCondFailures = 0;
    }

    ExecContext::Status status;

    int cpuId;

    // Index of hardware thread context on the CPU that this represents.
    int tid;

    Counter numInst;
    Stats::Scalar<> numInsts;
    Stats::Scalar<> numMemRefs;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    FunctionalMemory *mem;	// functional storage for process address space

#if FULL_SYSTEM
    Tick lastActivate;
    Tick lastSuspend;

    FunctionProfile *profile;
    ProfileNode *profileNode;
    Addr profilePC;

    EndQuiesceEvent *quiesceEvent;

#else
    Process *process;

    // Address space ID.  Note that this is used for TIMING cache
    // simulation only; all functional memory accesses should use
    // one of the FunctionalMemory pointers above.
    short asid;

#endif

    /**
     * Temporary storage to pass the source address from copy_load to
     * copy_store.
     * @todo Remove this temporary when we have a better way to do it.
     */
    Addr copySrcAddr;
    /**
     * Temp storage for the physical source address of a copy.
     * @todo Remove this temporary when we have a better way to do it.
     */
    Addr copySrcPhysAddr;

    /*
     * number of executed instructions, for matching with syscall trace
     * points in EIO files.
     */
    Counter funcExeInst;

    //
    // Count failed store conditionals so we can warn of apparent
    // application deadlock situations.
    unsigned storeCondFailures;
};

#endif // __CPU_THREAD_STATE_HH__
