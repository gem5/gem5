/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_O3_ALPHA_FULL_CPU_HH__
#define __CPU_O3_ALPHA_FULL_CPU_HH__

#include "arch/isa_traits.hh"
#include "cpu/exec_context.hh"
#include "cpu/o3/cpu.hh"
#include "sim/byteswap.hh"

template <class Impl>
class AlphaFullCPU : public FullO3CPU<Impl>
{
  protected:
    typedef TheISA::IntReg IntReg;
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MiscRegFile MiscRegFile;

  public:
    typedef O3ThreadState<Impl> ImplState;
    typedef O3ThreadState<Impl> Thread;
    typedef typename Impl::Params Params;

    /** Constructs an AlphaFullCPU with the given parameters. */
    AlphaFullCPU(Params *params);

    class AlphaXC : public ExecContext
    {
      public:
        AlphaFullCPU<Impl> *cpu;

        O3ThreadState<Impl> *thread;

        Tick lastActivate;
        Tick lastSuspend;

        Event *quiesceEvent;

        virtual BaseCPU *getCpuPtr() { return cpu; }

        virtual void setCpuId(int id) { cpu->cpu_id = id; }

        virtual int readCpuId() { return cpu->cpu_id; }

        virtual FunctionalMemory *getMemPtr() { return thread->mem; }

#if FULL_SYSTEM
        virtual System *getSystemPtr() { return cpu->system; }

        virtual PhysicalMemory *getPhysMemPtr() { return cpu->physmem; }

        virtual AlphaITB *getITBPtr() { return cpu->itb; }

        virtual AlphaDTB * getDTBPtr() { return cpu->dtb; }
#else
        virtual Process *getProcessPtr() { return thread->process; }
#endif

        virtual Status status() const { return thread->status(); }

        virtual void setStatus(Status new_status) { thread->setStatus(new_status); }

        /// Set the status to Active.  Optional delay indicates number of
        /// cycles to wait before beginning execution.
        virtual void activate(int delay = 1);

        /// Set the status to Suspended.
        virtual void suspend();

        /// Set the status to Unallocated.
        virtual void deallocate();

        /// Set the status to Halted.
        virtual void halt();

#if FULL_SYSTEM
        virtual void dumpFuncProfile();
#endif

        virtual void takeOverFrom(ExecContext *old_context);

        virtual void regStats(const std::string &name);

        virtual void serialize(std::ostream &os);
        virtual void unserialize(Checkpoint *cp, const std::string &section);

#if FULL_SYSTEM
        virtual Event *getQuiesceEvent();

        // Not necessarily the best location for these...
        // Having an extra function just to read these is obnoxious
        virtual Tick readLastActivate();
        virtual Tick readLastSuspend();

        virtual void profileClear();
        virtual void profileSample();
#endif

        virtual int getThreadNum() { return thread->tid; }

        // Also somewhat obnoxious.  Really only used for the TLB fault.
        // However, may be quite useful in SPARC.
        virtual TheISA::MachInst getInst();

        virtual void copyArchRegs(ExecContext *xc);

        virtual void clearArchRegs();

        //
        // New accessors for new decoder.
        //
        virtual uint64_t readIntReg(int reg_idx);

        virtual float readFloatRegSingle(int reg_idx);

        virtual double readFloatRegDouble(int reg_idx);

        virtual uint64_t readFloatRegInt(int reg_idx);

        virtual void setIntReg(int reg_idx, uint64_t val);

        virtual void setFloatRegSingle(int reg_idx, float val);

        virtual void setFloatRegDouble(int reg_idx, double val);

        virtual void setFloatRegInt(int reg_idx, uint64_t val);

        virtual uint64_t readPC()
        { return cpu->readPC(thread->tid); }

        virtual void setPC(uint64_t val);

        virtual uint64_t readNextPC()
        { return cpu->readNextPC(thread->tid); }

        virtual void setNextPC(uint64_t val);

        virtual MiscReg readMiscReg(int misc_reg)
        { return cpu->readMiscReg(misc_reg, thread->tid); }

        virtual MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
        { return cpu->readMiscRegWithEffect(misc_reg, fault, thread->tid); }

        virtual Fault setMiscReg(int misc_reg, const MiscReg &val);

        virtual Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val);

        // Also not necessarily the best location for these two.
        // Hopefully will go away once we decide upon where st cond
        // failures goes.
        virtual unsigned readStCondFailures() { return thread->storeCondFailures; }

        virtual void setStCondFailures(unsigned sc_failures) { thread->storeCondFailures = sc_failures; }

#if FULL_SYSTEM
        virtual bool inPalMode() { return TheISA::PcPAL(cpu->readPC(thread->tid)); }
#endif

        // Only really makes sense for old CPU model.  Still could be useful though.
        virtual bool misspeculating() { return false; }

#if !FULL_SYSTEM
        virtual IntReg getSyscallArg(int i);

        // used to shift args for indirect syscall
        virtual void setSyscallArg(int i, IntReg val);

        virtual void setSyscallReturn(SyscallReturn return_value);

        virtual void syscall() { return cpu->syscall(thread->tid); }

        // Same with st cond failures.
        virtual Counter readFuncExeInst() { return thread->funcExeInst; }
#endif
    };

    friend class AlphaXC;

    std::vector<AlphaXC *> xcProxies;

#if FULL_SYSTEM
    /** ITB pointer. */
    AlphaITB *itb;
    /** DTB pointer. */
    AlphaDTB *dtb;
#endif

    /** Registers statistics. */
    void regStats();

#if FULL_SYSTEM
    //Note that the interrupt stuff from the base CPU might be somewhat
    //ISA specific (ie NumInterruptLevels).  These functions might not
    //be needed in FullCPU though.
//    void post_interrupt(int int_num, int index);
//    void clear_interrupt(int int_num, int index);
//    void clear_interrupts();

    /** Translates instruction requestion. */
    Fault translateInstReq(MemReqPtr &req)
    {
        return itb->translate(req);
    }

    /** Translates data read request. */
    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dtb->translate(req, false);
    }

    /** Translates data write request. */
    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dtb->translate(req, true);
    }

#else
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

    /** Translates instruction requestion in syscall emulation mode. */
    Fault translateInstReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    /** Translates data read request in syscall emulation mode. */
    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    /** Translates data write request in syscall emulation mode. */
    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

#endif

    // Later on may want to remove this misc stuff from the regfile and
    // have it handled at this level.  This would be similar to moving certain
    // IPRs into the devices themselves.  Might prove to be an issue when
    // trying to rename source/destination registers...
    MiscReg readMiscReg(int misc_reg, unsigned tid);

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault, unsigned tid);

    Fault setMiscReg(int misc_reg, const MiscReg &val, unsigned tid);

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val, unsigned tid);

    void squashFromXC(unsigned tid);

#if FULL_SYSTEM
    void post_interrupt(int int_num, int index);

    int readIntrFlag();
    /** Sets the interrupt flags. */
    void setIntrFlag(int val);
    /** HW return from error interrupt. */
    Fault hwrei(unsigned tid);
    /** Returns if a specific PC is a PAL mode PC. */
    bool inPalMode(uint64_t PC)
    { return AlphaISA::PcPAL(PC); }

    /** Traps to handle given fault. */
    void trap(Fault fault, unsigned tid);
    bool simPalCheck(int palFunc);

    /** Processes any interrupts. */
    void processInterrupts();
#endif


#if !FULL_SYSTEM
    // Need to change these into regfile calls that directly set a certain
    // register.  Actually, these functions should handle most of this
    // functionality by themselves; should look up the rename and then
    // set the register.
    /** Gets a syscall argument. */
    IntReg getSyscallArg(int i, int tid);

    /** Used to shift args for indirect syscall. */
    void setSyscallArg(int i, IntReg val, int tid);

    /** Sets the return value of a syscall. */
    void setSyscallReturn(SyscallReturn return_value, int tid);

    /** Executes a syscall.
     * @todo: Determine if this needs to be virtual.
     */
    virtual void syscall(int thread_num);

#endif

  public:
#if FULL_SYSTEM
    /** Halts the CPU. */
    void halt() { panic("Halt not implemented!\n"); }
#endif

    /** Old CPU read from memory function. No longer used. */
    template <class T>
    Fault read(MemReqPtr &req, T &data)
    {
//	panic("CPU READ NOT IMPLEMENTED W/NEW MEMORY\n");
#if 0
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        if (req->flags & LOCKED) {
            req->xc->setMiscReg(TheISA::Lock_Addr_DepTag, req->paddr);
            req->xc->setMiscReg(TheISA::Lock_Flag_DepTag, true);
        }
#endif
#endif
        Fault error;
        if (req->flags & LOCKED) {
            lockAddr = req->paddr;
            lockFlag = true;
        }

        error = this->mem->read(req, data);
        data = gtoh(data);
        return error;
    }

    /** CPU read function, forwards read to LSQ. */
    template <class T>
    Fault read(MemReqPtr &req, T &data, int load_idx)
    {
        return this->iew.ldstQueue.read(req, data, load_idx);
    }

    /** Old CPU write to memory function. No longer used. */
    template <class T>
    Fault write(MemReqPtr &req, T &data)
    {
#if 0
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
        for (int i = 0; i < this->system->execContexts.size(); i++){
            xc = this->system->execContexts[i];
            if ((xc->readMiscReg(TheISA::Lock_Addr_DepTag) & ~0xf) ==
                (req->paddr & ~0xf)) {
                xc->setMiscReg(TheISA::Lock_Flag_DepTag, false);
            }
        }

#endif
#endif

        if (req->flags & LOCKED) {
            if (req->flags & UNCACHEABLE) {
                req->result = 2;
            } else {
                if (this->lockFlag/* && this->lockAddr == req->paddr*/) {
                    req->result=1;
                } else {
                    req->result = 0;
                }
            }
        }

        return this->mem->write(req, (T)htog(data));
    }

    /** CPU write function, forwards write to LSQ. */
    template <class T>
    Fault write(MemReqPtr &req, T &data, int store_idx)
    {
        return this->iew.ldstQueue.write(req, data, store_idx);
    }

    Addr lockAddr;
    bool lockFlag;
};

#endif // __CPU_O3_ALPHA_FULL_CPU_HH__
