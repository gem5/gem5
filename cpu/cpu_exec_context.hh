/*
 * Copyright (c) 2001-2006 The Regents of The University of Michigan
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

#ifndef __CPU_CPU_EXEC_CONTEXT_HH__
#define __CPU_CPU_EXEC_CONTEXT_HH__

#include "arch/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/exec_context.hh"
#include "mem/functional/functional.hh"
#include "mem/mem_req.hh"
#include "sim/byteswap.hh"
#include "sim/eventq.hh"
#include "sim/host.hh"
#include "sim/serialize.hh"

// forward declaration: see functional_memory.hh
class FunctionalMemory;
class PhysicalMemory;
class BaseCPU;

#if FULL_SYSTEM

#include "sim/system.hh"
#include "arch/tlb.hh"

class FunctionProfile;
class ProfileNode;
class MemoryController;

#else // !FULL_SYSTEM

#include "sim/process.hh"

#endif // FULL_SYSTEM

//
// The CPUExecContext object represents a functional context for
// instruction execution.  It incorporates everything required for
// architecture-level functional simulation of a single thread.
//

class CPUExecContext
{
  protected:
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscRegFile MiscRegFile;
    typedef TheISA::MiscReg MiscReg;
  public:
    typedef ExecContext::Status Status;

  private:
    Status _status;

  public:
    Status status() const { return _status; }

    void setStatus(Status newStatus) { _status = newStatus; }

    /// Set the status to Active.  Optional delay indicates number of
    /// cycles to wait before beginning execution.
    void activate(int delay = 1);

    /// Set the status to Suspended.
    void suspend();

    /// Set the status to Unallocated.
    void deallocate();

    /// Set the status to Halted.
    void halt();

  protected:
    RegFile regs;	// correct-path register context

  public:
    // pointer to CPU associated with this context
    BaseCPU *cpu;

    ProxyExecContext<CPUExecContext> *proxy;

    // Current instruction
    MachInst inst;

    // Index of hardware thread context on the CPU that this represents.
    int thread_num;

    // ID of this context w.r.t. the System or Process object to which
    // it belongs.  For full-system mode, this is the system CPU ID.
    int cpu_id;

    Tick lastActivate;
    Tick lastSuspend;

#if FULL_SYSTEM
    FunctionalMemory *mem;
    AlphaITB *itb;
    AlphaDTB *dtb;
    System *system;

    // the following two fields are redundant, since we can always
    // look them up through the system pointer, but we'll leave them
    // here for now for convenience
    MemoryController *memctrl;
    PhysicalMemory *physmem;

    FunctionProfile *profile;
    ProfileNode *profileNode;
    Addr profilePC;
    void dumpFuncProfile();

    /** Event for timing out quiesce instruction */
    struct EndQuiesceEvent : public Event
    {
        /** A pointer to the execution context that is quiesced */
        CPUExecContext *cpuXC;

        EndQuiesceEvent(CPUExecContext *_cpuXC);

        /** Event process to occur at interrupt*/
        virtual void process();

        /** Event description */
        virtual const char *description();
    };
    EndQuiesceEvent quiesceEvent;

    Event *getQuiesceEvent() { return &quiesceEvent; }

    Tick readLastActivate() { return lastActivate; }

    Tick readLastSuspend() { return lastSuspend; }

#else
    Process *process;

    FunctionalMemory *mem;	// functional storage for process address space

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
    Counter func_exe_inst;

    //
    // Count failed store conditionals so we can warn of apparent
    // application deadlock situations.
    unsigned storeCondFailures;

    // constructor: initialize context from given process structure
#if FULL_SYSTEM
    CPUExecContext(BaseCPU *_cpu, int _thread_num, System *_system,
                   AlphaITB *_itb, AlphaDTB *_dtb, FunctionalMemory *_dem);
#else
    CPUExecContext(BaseCPU *_cpu, int _thread_num, Process *_process, int _asid);
    CPUExecContext(BaseCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                   int _asid);
    // Constructor to use XC to pass reg file around.  Not used for anything
    // else.
    CPUExecContext(RegFile *regFile);
#endif
    virtual ~CPUExecContext();

    virtual void takeOverFrom(ExecContext *oldContext);

    void regStats(const std::string &name);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

    BaseCPU *getCpuPtr() { return cpu; }

    ExecContext *getProxy() { return proxy; }

    int getThreadNum() { return thread_num; }

#if FULL_SYSTEM
    System *getSystemPtr() { return system; }

    PhysicalMemory *getPhysMemPtr() { return physmem; }

    AlphaITB *getITBPtr() { return itb; }

    AlphaDTB *getDTBPtr() { return dtb; }

    bool validInstAddr(Addr addr) { return true; }
    bool validDataAddr(Addr addr) { return true; }
    int getInstAsid() { return regs.instAsid(); }
    int getDataAsid() { return regs.dataAsid(); }

    Fault translateInstReq(MemReqPtr &req)
    {
        return itb->translate(req);
    }

    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dtb->translate(req, false);
    }

    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dtb->translate(req, true);
    }

#else
    Process *getProcessPtr() { return process; }

    bool validInstAddr(Addr addr)
    { return process->validInstAddr(addr); }

    bool validDataAddr(Addr addr)
    { return process->validDataAddr(addr); }

    int getInstAsid() { return asid; }
    int getDataAsid() { return asid; }

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

#endif

    template <class T>
    Fault read(MemReqPtr &req, T &data)
    {
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        if (req->flags & LOCKED) {
            req->xc->setMiscReg(TheISA::Lock_Addr_DepTag, req->paddr);
            req->xc->setMiscReg(TheISA::Lock_Flag_DepTag, true);
        }
#endif

        Fault error;
        error = mem->read(req, data);
        data = LittleEndianGuest::gtoh(data);
        return error;
    }

    template <class T>
    Fault write(MemReqPtr &req, T &data)
    {
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        ExecContext *xc;

        // If this is a store conditional, act appropriately
        if (req->flags & LOCKED) {
            xc = req->xc;

            if (req->flags & UNCACHEABLE) {
                // Don't update result register (see stq_c in isa_desc)
                req->result = 2;
                xc->setStCondFailures(0);//Needed? [RGD]
            } else {
                bool lock_flag = xc->readMiscReg(TheISA::Lock_Flag_DepTag);
                Addr lock_addr = xc->readMiscReg(TheISA::Lock_Addr_DepTag);
                req->result = lock_flag;
                if (!lock_flag ||
                    ((lock_addr & ~0xf) != (req->paddr & ~0xf))) {
                    xc->setMiscReg(TheISA::Lock_Flag_DepTag, false);
                    xc->setStCondFailures(xc->readStCondFailures() + 1);
                    if (((xc->readStCondFailures()) % 100000) == 0) {
                        std::cerr << "Warning: "
                                  << xc->readStCondFailures()
                                  << " consecutive store conditional failures "
                                  << "on cpu " << req->xc->readCpuId()
                                  << std::endl;
                    }
                    return NoFault;
                }
                else xc->setStCondFailures(0);
            }
        }

        // Need to clear any locked flags on other proccessors for
        // this address.  Only do this for succsful Store Conditionals
        // and all other stores (WH64?).  Unsuccessful Store
        // Conditionals would have returned above, and wouldn't fall
        // through.
        for (int i = 0; i < system->execContexts.size(); i++){
            xc = system->execContexts[i];
            if ((xc->readMiscReg(TheISA::Lock_Addr_DepTag) & ~0xf) ==
                (req->paddr & ~0xf)) {
                xc->setMiscReg(TheISA::Lock_Flag_DepTag, false);
            }
        }

#endif
        return mem->write(req, (T)LittleEndianGuest::htog(data));
    }

    virtual bool misspeculating();


    MachInst getInst() { return inst; }

    void setInst(MachInst new_inst)
    {
        inst = new_inst;
    }

    Fault instRead(MemReqPtr &req)
    {
        return mem->read(req, inst);
    }

    void setCpuId(int id) { cpu_id = id; }

    int readCpuId() { return cpu_id; }

    FunctionalMemory *getMemPtr() { return mem; }

    void copyArchRegs(ExecContext *xc);

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    {
        return regs.intRegFile[reg_idx];
    }

    float readFloatRegSingle(int reg_idx)
    {
        return (float)regs.floatRegFile.d[reg_idx];
    }

    double readFloatRegDouble(int reg_idx)
    {
        return regs.floatRegFile.d[reg_idx];
    }

    uint64_t readFloatRegInt(int reg_idx)
    {
        return regs.floatRegFile.q[reg_idx];
    }

    void setIntReg(int reg_idx, uint64_t val)
    {
        regs.intRegFile[reg_idx] = val;
    }

    void setFloatRegSingle(int reg_idx, float val)
    {
        regs.floatRegFile.d[reg_idx] = (double)val;
    }

    void setFloatRegDouble(int reg_idx, double val)
    {
        regs.floatRegFile.d[reg_idx] = val;
    }

    void setFloatRegInt(int reg_idx, uint64_t val)
    {
        regs.floatRegFile.q[reg_idx] = val;
    }

    uint64_t readPC()
    {
        return regs.pc;
    }

    void setPC(uint64_t val)
    {
        regs.pc = val;
    }

    uint64_t readNextPC()
    {
        return regs.npc;
    }

    void setNextPC(uint64_t val)
    {
        regs.npc = val;
    }

    MiscReg readMiscReg(int misc_reg)
    {
        return regs.miscRegs.readReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return regs.miscRegs.readRegWithEffect(misc_reg, fault, proxy);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setRegWithEffect(misc_reg, val, proxy);
    }

    unsigned readStCondFailures() { return storeCondFailures; }

    void setStCondFailures(unsigned sc_failures)
    { storeCondFailures = sc_failures; }

    void clearArchRegs() { memset(&regs, 0, sizeof(regs)); }

#if FULL_SYSTEM
    int readIntrFlag() { return regs.intrflag; }
    void setIntrFlag(int val) { regs.intrflag = val; }
    Fault hwrei();
    bool inPalMode() { return AlphaISA::PcPAL(regs.pc); }
    bool simPalCheck(int palFunc);
#endif

#if !FULL_SYSTEM
    TheISA::IntReg getSyscallArg(int i)
    {
        return regs.intRegFile[TheISA::ArgumentReg0 + i];
    }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, TheISA::IntReg val)
    {
        regs.intRegFile[TheISA::ArgumentReg0 + i] = val;
    }

    void setSyscallReturn(SyscallReturn return_value)
    {
        // check for error condition.  Alpha syscall convention is to
        // indicate success/failure in reg a3 (r19) and put the
        // return value itself in the standard return value reg (v0).
        const int RegA3 = 19;	// only place this is used
        if (return_value.successful()) {
            // no error
            regs.intRegFile[RegA3] = 0;
            regs.intRegFile[TheISA::ReturnValueReg] = return_value.value();
        } else {
            // got an error, return details
            regs.intRegFile[RegA3] = (TheISA::IntReg) -1;
            regs.intRegFile[TheISA::ReturnValueReg] = -return_value.value();
        }
    }

    void syscall()
    {
        process->syscall(proxy);
    }

    Counter readFuncExeInst() { return func_exe_inst; }

    void setFuncExeInst(Counter new_val) { func_exe_inst = new_val; }
#endif
};


// for non-speculative execution context, spec_mode is always false
inline bool
CPUExecContext::misspeculating()
{
    return false;
}

#endif // __CPU_CPU_EXEC_CONTEXT_HH__
