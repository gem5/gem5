#ifndef __CPU_CHECKER_EXEC_CONTEXT_HH__
#define __CPU_CHECKER_EXEC_CONTEXT_HH__

#include "cpu/checker/cpu.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/exec_context.hh"

class EndQuiesceEvent;

template <class XC>
class CheckerExecContext : public ExecContext
{
  public:
    CheckerExecContext(XC *actual_xc,
                       CheckerCPU *checker_cpu)
        : actualXC(actual_xc), checkerXC(checker_cpu->cpuXC), checkerCPU(checker_cpu)
    { }

  private:
    XC *actualXC;
    CPUExecContext *checkerXC;
    CheckerCPU *checkerCPU;

  public:

    BaseCPU *getCpuPtr() { return actualXC->getCpuPtr(); }

    void setCpuId(int id)
    {
        actualXC->setCpuId(id);
        checkerXC->setCpuId(id);
    }

    int readCpuId() { return actualXC->readCpuId(); }

    FunctionalMemory *getMemPtr() { return actualXC->getMemPtr(); }

#if FULL_SYSTEM
    System *getSystemPtr() { return actualXC->getSystemPtr(); }

    PhysicalMemory *getPhysMemPtr() { return actualXC->getPhysMemPtr(); }

    AlphaITB *getITBPtr() { return actualXC->getITBPtr(); }

    AlphaDTB *getDTBPtr() { return actualXC->getDTBPtr(); }
#else
    Process *getProcessPtr() { return actualXC->getProcessPtr(); }
#endif

    Status status() const { return actualXC->status(); }

    void setStatus(Status new_status)
    { actualXC->setStatus(new_status);
    checkerXC->setStatus(new_status); }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1) { actualXC->activate(delay); }

    /// Set the status to Suspended.
    void suspend() { actualXC->suspend(); }

    /// Set the status to Unallocated.
    void deallocate() { actualXC->deallocate(); }

    /// Set the status to Halted.
    void halt() { actualXC->halt(); }

#if FULL_SYSTEM
    void dumpFuncProfile() { actualXC->dumpFuncProfile(); }
#endif

    void takeOverFrom(ExecContext *oldContext)
    {
        actualXC->takeOverFrom(oldContext);
        checkerXC->takeOverFrom(oldContext);
    }

    void regStats(const std::string &name) { actualXC->regStats(name); }

    void serialize(std::ostream &os) { actualXC->serialize(os); }
    void unserialize(Checkpoint *cp, const std::string &section)
    { actualXC->unserialize(cp, section); }

#if FULL_SYSTEM
    EndQuiesceEvent *getQuiesceEvent() { return actualXC->getQuiesceEvent(); }

    Tick readLastActivate() { return actualXC->readLastActivate(); }
    Tick readLastSuspend() { return actualXC->readLastSuspend(); }

    void profileClear() { return actualXC->profileClear(); }
    void profileSample() { return actualXC->profileSample(); }
#endif

    int getThreadNum() { return actualXC->getThreadNum(); }

    // @todo: Do I need this?
    MachInst getInst() { return actualXC->getInst(); }

    // @todo: Do I need this?
    void copyArchRegs(ExecContext *xc)
    {
        actualXC->copyArchRegs(xc);
        checkerXC->copyArchRegs(xc);
    }

    void clearArchRegs()
    {
        actualXC->clearArchRegs();
        checkerXC->clearArchRegs();
    }

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    { return actualXC->readIntReg(reg_idx); }

    float readFloatRegSingle(int reg_idx)
    { return actualXC->readFloatRegSingle(reg_idx); }

    double readFloatRegDouble(int reg_idx)
    { return actualXC->readFloatRegDouble(reg_idx); }

    uint64_t readFloatRegInt(int reg_idx)
    { return actualXC->readFloatRegInt(reg_idx); }

    void setIntReg(int reg_idx, uint64_t val)
    {
        actualXC->setIntReg(reg_idx, val);
        checkerXC->setIntReg(reg_idx, val);
    }

    void setFloatRegSingle(int reg_idx, float val)
    {
        actualXC->setFloatRegSingle(reg_idx, val);
        checkerXC->setFloatRegSingle(reg_idx, val);
    }

    void setFloatRegDouble(int reg_idx, double val)
    {
        actualXC->setFloatRegDouble(reg_idx, val);
        checkerXC->setFloatRegSingle(reg_idx, val);
    }

    void setFloatRegInt(int reg_idx, uint64_t val)
    {
        actualXC->setFloatRegInt(reg_idx, val);
        checkerXC->setFloatRegInt(reg_idx, val);
    }

    uint64_t readPC() { return actualXC->readPC(); }

    void setPC(uint64_t val)
    {
        actualXC->setPC(val);
        checkerXC->setPC(val);
        checkerCPU->recordPCChange(val);
    }

    uint64_t readNextPC() { return actualXC->readNextPC(); }

    void setNextPC(uint64_t val)
    {
        actualXC->setNextPC(val);
        checkerXC->setNextPC(val);
        checkerCPU->recordNextPCChange(val);
    }

    MiscReg readMiscReg(int misc_reg)
    { return actualXC->readMiscReg(misc_reg); }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    { return actualXC->readMiscRegWithEffect(misc_reg, fault); }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        checkerXC->setMiscReg(misc_reg, val);
        return actualXC->setMiscReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        checkerXC->setMiscRegWithEffect(misc_reg, val);
        return actualXC->setMiscRegWithEffect(misc_reg, val);
    }

    unsigned readStCondFailures()
    { return actualXC->readStCondFailures(); }

    void setStCondFailures(unsigned sc_failures)
    {
        checkerXC->setStCondFailures(sc_failures);
        actualXC->setStCondFailures(sc_failures);
    }
#if FULL_SYSTEM
    bool inPalMode() { return actualXC->inPalMode(); }
#endif

    // @todo: Fix this!
    bool misspeculating() { return actualXC->misspeculating(); }

#if !FULL_SYSTEM
    IntReg getSyscallArg(int i) { return actualXC->getSyscallArg(i); }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, IntReg val)
    {
        checkerXC->setSyscallArg(i, val);
        actualXC->setSyscallArg(i, val);
    }

    void setSyscallReturn(SyscallReturn return_value)
    {
        checkerXC->setSyscallReturn(return_value);
        actualXC->setSyscallReturn(return_value);
    }

//    void syscall() { actualXC->syscall(); }

    Counter readFuncExeInst() { return actualXC->readFuncExeInst(); }
#endif
};

#endif // __CPU_CHECKER_EXEC_CONTEXT_HH__
