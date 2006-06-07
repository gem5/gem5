#include "base/output.hh"
#include "cpu/profile.hh"
#include "cpu/thread_state.hh"

#if FULL_SYSTEM
ThreadState::ThreadState(int _cpuId, int _tid)
    : cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
      profile(NULL), profileNode(NULL), profilePC(0), quiesceEvent(NULL),
      funcExeInst(0), storeCondFailures(0)
#else
ThreadState::ThreadState(int _cpuId, int _tid, MemObject *mem,
                         Process *_process, short _asid)
    : cpuId(_cpuId), tid(_tid), lastActivate(0), lastSuspend(0),
      process(_process), asid(_asid),
      funcExeInst(0), storeCondFailures(0)
#endif
{
#if !FULL_SYSTEM
        /* Use this port to for syscall emulation writes to memory. */
        Port *mem_port;
        port = new TranslatingPort(csprintf("%d-funcport",
                                            tid),
                                   process->pTable, false);
        mem_port = mem->getPort("functional");
        mem_port->setPeer(port);
        port->setPeer(mem_port);
#endif
}

#if FULL_SYSTEM

void
ThreadState::profileClear()
{
    if (profile)
        profile->clear();
}

void
ThreadState::profileSample()
{
    if (profile)
        profile->sample(profileNode, profilePC);
}

#endif
