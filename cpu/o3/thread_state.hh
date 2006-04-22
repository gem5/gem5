
#ifndef __CPU_O3_THREAD_STATE_HH__
#define __CPU_O3_THREAD_STATE_HH__

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "cpu/exec_context.hh"
#include "cpu/thread_state.hh"

class Event;
class Process;

#if FULL_SYSTEM
class EndQuiesceEvent;
class FunctionProfile;
class ProfileNode;
#else
class Process;
class FunctionalMemory;
#endif

// In the new CPU case this may be quite small...It depends on what I define
// ThreadState to be.  Currently it's only the state that exists within
// ExecContext basically.  Leaves the interface and manipulation up to the
// CPU.  Not sure this is useful/flexible...probably can be if I can avoid
// including state here that parts of the pipeline can't modify directly,
// or at least don't let them.  The only problem is for state that's needed
// per thread, per structure.  I.e. rename table, memreqs.
// On the other hand, it might be nice to not have to pay the extra pointer
// lookup to get frequently used state such as a memreq (that isn't used much
// elsewhere)...

// Maybe this ozone thread state should only really have committed state?
// I need to think about why I'm using this and what it's useful for.  Clearly
// has benefits for SMT; basically serves same use as CPUExecContext.
// Makes the ExecContext proxy easier.  Gives organization/central access point
// to state of a thread that can be accessed normally (i.e. not in-flight
// stuff within a OoO processor).  Does this need an XC proxy within it?
template <class Impl>
struct O3ThreadState : public ThreadState {
    typedef ExecContext::Status Status;
    typedef typename Impl::FullCPU FullCPU;

    Status _status;

    // Current instruction?
    TheISA::MachInst inst;
  private:
    FullCPU *cpu;
  public:

    bool inSyscall;

    bool trapPending;

#if FULL_SYSTEM
    O3ThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem)
        : ThreadState(-1, _thread_num, _mem),
          inSyscall(0), trapPending(0)
    { }
#else
    O3ThreadState(FullCPU *_cpu, int _thread_num, Process *_process, int _asid)
        : ThreadState(-1, _thread_num, NULL, _process, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    { }

    O3ThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                  int _asid)
        : ThreadState(-1, _thread_num, _mem, NULL, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    { }
#endif

    ExecContext *xcProxy;

    ExecContext *getXCProxy() { return xcProxy; }

    Status status() const { return _status; }

    void setStatus(Status new_status) { _status = new_status; }

#if !FULL_SYSTEM

    Fault dummyTranslation(MemReqPtr &req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return NoFault;
    }
    Fault translateInstReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    bool validInstAddr(Addr addr)
    { return process->validInstAddr(addr); }

    bool validDataAddr(Addr addr)
    { return process->validDataAddr(addr); }
#else
    Fault translateInstReq(MemReqPtr &req)
    {
        return cpu->itb->translate(req);
    }

    Fault translateDataReadReq(MemReqPtr &req)
    {
        return cpu->dtb->translate(req, false);
    }

    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return cpu->dtb->translate(req, true);
    }
#endif

    bool misspeculating() { return false; }

    void setInst(TheISA::MachInst _inst) { inst = _inst; }

    Counter readFuncExeInst() { return funcExeInst; }

    void setFuncExeInst(Counter new_val) { funcExeInst = new_val; }

#if !FULL_SYSTEM
    void syscall() { process->syscall(xcProxy); }
#endif
};

#endif // __CPU_O3_THREAD_STATE_HH__
