
#ifndef __CPU_OZONE_THREAD_STATE_HH__
#define __CPU_OZONE_THREAD_STATE_HH__

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

// Maybe this ozone thread state should only really have committed state?
// I need to think about why I'm using this and what it's useful for.  Clearly
// has benefits for SMT; basically serves same use as CPUExecContext.
// Makes the ExecContext proxy easier.  Gives organization/central access point
// to state of a thread that can be accessed normally (i.e. not in-flight
// stuff within a OoO processor).  Does this need an XC proxy within it?
template <class Impl>
struct OzoneThreadState : public ThreadState {
    typedef typename ExecContext::Status Status;
    typedef typename Impl::FullCPU FullCPU;
    typedef TheISA::MiscReg MiscReg;

#if FULL_SYSTEM
    OzoneThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem)
        : ThreadState(-1, _thread_num, _mem),
          inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }
#else
    OzoneThreadState(FullCPU *_cpu, int _thread_num, Process *_process, int _asid)
        : ThreadState(-1, _thread_num, NULL, _process, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }

    OzoneThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                     int _asid)
        : ThreadState(-1, _thread_num, _mem, NULL, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    {
        memset(&regs, 0, sizeof(TheISA::RegFile));
    }
#endif

    Status _status;

    Status status() const { return _status; }

    void setStatus(Status new_status) { _status = new_status; }

    RenameTable<Impl> renameTable; // Should I include backend and frontend
    // tables here?  For the ozone CPU, maybe, for the new full CPU, probably
    // not...you wouldn't want threads just accessing the backend/frontend
    // rename tables.
    Addr PC; // What should these be set to?  Probably the committed ones.
    Addr nextPC;

    // Current instruction?
    TheISA::MachInst inst;

    TheISA::RegFile regs;
    // Front end?  Back end?
//    MemReqPtr memReq;

    typename Impl::FullCPU *cpu;

    bool inSyscall;

    bool trapPending;

    ExecContext *xcProxy;

    ExecContext *getXCProxy() { return xcProxy; }

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

    MiscReg readMiscReg(int misc_reg)
    {
        return regs.miscRegs.readReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return regs.miscRegs.readRegWithEffect(misc_reg, fault, xcProxy);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setRegWithEffect(misc_reg, val, xcProxy);
    }

    uint64_t readPC()
    { return PC; }

    void setPC(uint64_t val)
    { PC = val; }

    uint64_t readNextPC()
    { return nextPC; }

    void setNextPC(uint64_t val)
    { nextPC = val; }

    bool misspeculating() { return false; }

    void setInst(TheISA::MachInst _inst) { inst = _inst; }

    Counter readFuncExeInst() { return funcExeInst; }

    void setFuncExeInst(Counter new_val) { funcExeInst = new_val; }
};

#endif // __CPU_OZONE_THREAD_STATE_HH__
