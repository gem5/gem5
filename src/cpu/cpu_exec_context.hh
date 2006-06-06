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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __CPU_CPU_EXEC_CONTEXT_HH__
#define __CPU_CPU_EXEC_CONTEXT_HH__

#include "arch/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/thread_context.hh"
#include "mem/physical.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/eventq.hh"
#include "sim/host.hh"
#include "sim/serialize.hh"

class BaseCPU;

#if FULL_SYSTEM

#include "sim/system.hh"
#include "arch/tlb.hh"

class FunctionProfile;
class ProfileNode;
class FunctionalPort;
class PhysicalPort;


namespace Kernel {
    class Statistics;
};

#else // !FULL_SYSTEM

#include "sim/process.hh"
#include "mem/page_table.hh"
class TranslatingPort;


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
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
  public:
    typedef ThreadContext::Status Status;

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

    ProxyThreadContext<CPUExecContext> *tc;

    // Current instruction
    MachInst inst;

    // Index of hardware thread context on the CPU that this represents.
    int thread_num;

    // ID of this context w.r.t. the System or Process object to which
    // it belongs.  For full-system mode, this is the system CPU ID.
    int cpu_id;

    Tick lastActivate;
    Tick lastSuspend;

    System *system;


#if FULL_SYSTEM
    AlphaITB *itb;
    AlphaDTB *dtb;

    /** A functional port outgoing only for functional accesses to physical
     * addresses.*/
    FunctionalPort *physPort;

    /** A functional port, outgoing only, for functional accesse to virtual
     * addresses. That doen't require execution context information */
    VirtualPort *virtPort;

    FunctionProfile *profile;
    ProfileNode *profileNode;
    Addr profilePC;
    void dumpFuncProfile();

    EndQuiesceEvent *quiesceEvent;

    EndQuiesceEvent *getQuiesceEvent() { return quiesceEvent; }

    Tick readLastActivate() { return lastActivate; }

    Tick readLastSuspend() { return lastSuspend; }

    void profileClear();

    void profileSample();

    Kernel::Statistics *getKernelStats() { return kernelStats; }

    Kernel::Statistics *kernelStats;
#else
    /// Port that syscalls can use to access memory (provides translation step).
    TranslatingPort *port;

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
    Counter func_exe_inst;

    //
    // Count failed store conditionals so we can warn of apparent
    // application deadlock situations.
    unsigned storeCondFailures;

    // constructor: initialize context from given process structure
#if FULL_SYSTEM
    CPUExecContext(BaseCPU *_cpu, int _thread_num, System *_system,
                   AlphaITB *_itb, AlphaDTB *_dtb,
                   bool use_kernel_stats = true);
#else
    CPUExecContext(BaseCPU *_cpu, int _thread_num, Process *_process, int _asid,
            MemObject *memobj);
    // Constructor to use XC to pass reg file around.  Not used for anything
    // else.
    CPUExecContext(RegFile *regFile);
#endif
    virtual ~CPUExecContext();

    virtual void takeOverFrom(ThreadContext *oldContext);

    void regStats(const std::string &name);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

    BaseCPU *getCpuPtr() { return cpu; }

    ThreadContext *getTC() { return tc; }

    int getThreadNum() { return thread_num; }

#if FULL_SYSTEM
    System *getSystemPtr() { return system; }

    AlphaITB *getITBPtr() { return itb; }

    AlphaDTB *getDTBPtr() { return dtb; }

    int getInstAsid() { return regs.instAsid(); }
    int getDataAsid() { return regs.dataAsid(); }

    Fault translateInstReq(RequestPtr &req)
    {
        return itb->translate(req, tc);
    }

    Fault translateDataReadReq(RequestPtr &req)
    {
        return dtb->translate(req, tc, false);
    }

    Fault translateDataWriteReq(RequestPtr &req)
    {
        return dtb->translate(req, tc, true);
    }

    FunctionalPort *getPhysPort() { return physPort; }

    /** Return a virtual port. If no thread context is specified then a static
     * port is returned. Otherwise a port is created and returned. It must be
     * deleted by deleteVirtPort(). */
    VirtualPort *getVirtPort(ThreadContext *tc);

    void delVirtPort(VirtualPort *vp);

#else
    TranslatingPort *getMemPort() { return port; }

    Process *getProcessPtr() { return process; }

    int getInstAsid() { return asid; }
    int getDataAsid() { return asid; }

    Fault translateInstReq(RequestPtr &req)
    {
        return process->pTable->translate(req);
    }

    Fault translateDataReadReq(RequestPtr &req)
    {
        return process->pTable->translate(req);
    }

    Fault translateDataWriteReq(RequestPtr &req)
    {
        return process->pTable->translate(req);
    }

#endif

/*
    template <class T>
    Fault read(RequestPtr &req, T &data)
    {
#if FULL_SYSTEM && THE_ISA == ALPHA_ISA
        if (req->flags & LOCKED) {
            req->xc->setMiscReg(TheISA::Lock_Addr_DepTag, req->paddr);
            req->xc->setMiscReg(TheISA::Lock_Flag_DepTag, true);
        }
#endif

        Fault error;
        error = mem->prot_read(req->paddr, data, req->size);
        data = LittleEndianGuest::gtoh(data);
        return error;
    }

    template <class T>
    Fault write(RequestPtr &req, T &data)
    {
#if FULL_SYSTEM && THE_ISA == ALPHA_ISA
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
        return mem->prot_write(req->paddr, (T)htog(data), req->size);
    }
*/
    virtual bool misspeculating();


    MachInst getInst() { return inst; }

    void setInst(MachInst new_inst)
    {
        inst = new_inst;
    }

    Fault instRead(RequestPtr &req)
    {
        panic("instRead not implemented");
        // return funcPhysMem->read(req, inst);
        return NoFault;
    }

    void setCpuId(int id) { cpu_id = id; }

    int readCpuId() { return cpu_id; }

    void copyArchRegs(ThreadContext *tc);

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx)
    {
        return regs.readIntReg(reg_idx);
    }

    FloatReg readFloatReg(int reg_idx, int width)
    {
        return regs.readFloatReg(reg_idx, width);
    }

    FloatReg readFloatReg(int reg_idx)
    {
        return regs.readFloatReg(reg_idx);
    }

    FloatRegBits readFloatRegBits(int reg_idx, int width)
    {
        return regs.readFloatRegBits(reg_idx, width);
    }

    FloatRegBits readFloatRegBits(int reg_idx)
    {
        return regs.readFloatRegBits(reg_idx);
    }

    void setIntReg(int reg_idx, uint64_t val)
    {
        regs.setIntReg(reg_idx, val);
    }

    void setFloatReg(int reg_idx, FloatReg val, int width)
    {
        regs.setFloatReg(reg_idx, val, width);
    }

    void setFloatReg(int reg_idx, FloatReg val)
    {
        regs.setFloatReg(reg_idx, val);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val, int width)
    {
        regs.setFloatRegBits(reg_idx, val, width);
    }

    void setFloatRegBits(int reg_idx, FloatRegBits val)
    {
        regs.setFloatRegBits(reg_idx, val);
    }

    uint64_t readPC()
    {
        return regs.readPC();
    }

    void setPC(uint64_t val)
    {
        regs.setPC(val);
    }

    uint64_t readNextPC()
    {
        return regs.readNextPC();
    }

    void setNextPC(uint64_t val)
    {
        regs.setNextPC(val);
    }

    uint64_t readNextNPC()
    {
        return regs.readNextNPC();
    }

    void setNextNPC(uint64_t val)
    {
        regs.setNextNPC(val);
    }


    MiscReg readMiscReg(int misc_reg)
    {
        return regs.readMiscReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return regs.readMiscRegWithEffect(misc_reg, fault, tc);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.setMiscReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return regs.setMiscRegWithEffect(misc_reg, val, tc);
    }

    unsigned readStCondFailures() { return storeCondFailures; }

    void setStCondFailures(unsigned sc_failures)
    { storeCondFailures = sc_failures; }

    void clearArchRegs() { regs.clear(); }

#if FULL_SYSTEM
    int readIntrFlag() { return regs.intrflag; }
    void setIntrFlag(int val) { regs.intrflag = val; }
    Fault hwrei();
    bool inPalMode() { return AlphaISA::PcPAL(regs.readPC()); }
    bool simPalCheck(int palFunc);
#endif

#if !FULL_SYSTEM
    TheISA::IntReg getSyscallArg(int i)
    {
        return regs.readIntReg(TheISA::ArgumentReg0 + i);
    }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, TheISA::IntReg val)
    {
        regs.setIntReg(TheISA::ArgumentReg0 + i, val);
    }

    void setSyscallReturn(SyscallReturn return_value)
    {
        TheISA::setSyscallReturn(return_value, &regs);
    }

    void syscall(int64_t callnum)
    {
        process->syscall(callnum, tc);
    }

    Counter readFuncExeInst() { return func_exe_inst; }

    void setFuncExeInst(Counter new_val) { func_exe_inst = new_val; }
#endif

    void changeRegFileContext(RegFile::ContextParam param,
            RegFile::ContextVal val)
    {
        regs.changeContext(param, val);
    }
};


// for non-speculative execution context, spec_mode is always false
inline bool
CPUExecContext::misspeculating()
{
    return false;
}

#endif // __CPU_CPU_EXEC_CONTEXT_HH__
