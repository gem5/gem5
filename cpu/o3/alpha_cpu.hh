/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

class EndQuiesceEvent;
namespace Kernel {
    class Statistics;
};

/**
 * AlphaFullCPU class.  Derives from the FullO3CPU class, and
 * implements all ISA and implementation specific functions of the
 * CPU.  This is the CPU class that is used for the SimObjects, and is
 * what is given to the DynInsts.  Most of its state exists in the
 * FullO3CPU; the state is has is mainly for ISA specific
 * functionality.
 */
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

    /**
     * Derived ExecContext class for use with the AlphaFullCPU.  It
     * provides the interface for any external objects to access a
     * single thread's state and some general CPU state.  Any time
     * external objects try to update state through this interface,
     * the CPU will create an event to squash all in-flight
     * instructions in order to ensure state is maintained correctly.
     */
    class AlphaXC : public ExecContext
    {
      public:
        /** Pointer to the CPU. */
        AlphaFullCPU<Impl> *cpu;

        /** Pointer to the thread state that this XC corrseponds to. */
        O3ThreadState<Impl> *thread;

        /** Returns a pointer to this CPU. */
        virtual BaseCPU *getCpuPtr() { return cpu; }

        /** Sets this CPU's ID. */
        virtual void setCpuId(int id) { cpu->cpu_id = id; }

        /** Reads this CPU's ID. */
        virtual int readCpuId() { return cpu->cpu_id; }

        /** Returns a pointer to functional memory. */
        virtual FunctionalMemory *getMemPtr() { return thread->mem; }

#if FULL_SYSTEM
        /** Returns a pointer to the system. */
        virtual System *getSystemPtr() { return cpu->system; }

        /** Returns a pointer to physical memory. */
        virtual PhysicalMemory *getPhysMemPtr() { return cpu->physmem; }

        /** Returns a pointer to the ITB. */
        virtual AlphaITB *getITBPtr() { return cpu->itb; }

        /** Returns a pointer to the DTB. */
        virtual AlphaDTB *getDTBPtr() { return cpu->dtb; }

        /** Returns a pointer to this thread's kernel statistics. */
        virtual Kernel::Statistics *getKernelStats()
        { return thread->kernelStats; }
#else
        /** Returns a pointer to this thread's process. */
        virtual Process *getProcessPtr() { return thread->process; }
#endif
        /** Returns this thread's status. */
        virtual Status status() const { return thread->status(); }

        /** Sets this thread's status. */
        virtual void setStatus(Status new_status)
        { thread->setStatus(new_status); }

        /** Set the status to Active.  Optional delay indicates number of
         * cycles to wait before beginning execution. */
        virtual void activate(int delay = 1);

        /** Set the status to Suspended. */
        virtual void suspend();

        /** Set the status to Unallocated. */
        virtual void deallocate();

        /** Set the status to Halted. */
        virtual void halt();

#if FULL_SYSTEM
        /** Dumps the function profiling information.
         * @todo: Implement.
         */
        virtual void dumpFuncProfile();
#endif
        /** Takes over execution of a thread from another CPU. */
        virtual void takeOverFrom(ExecContext *old_context);

        /** Registers statistics associated with this XC. */
        virtual void regStats(const std::string &name);

        /** Serializes state. */
        virtual void serialize(std::ostream &os);
        /** Unserializes state. */
        virtual void unserialize(Checkpoint *cp, const std::string &section);

#if FULL_SYSTEM
        /** Returns pointer to the quiesce event. */
        virtual EndQuiesceEvent *getQuiesceEvent();

        /** Reads the last tick that this thread was activated on. */
        virtual Tick readLastActivate();
        /** Reads the last tick that this thread was suspended on. */
        virtual Tick readLastSuspend();

        /** Clears the function profiling information. */
        virtual void profileClear();
        /** Samples the function profiling information. */
        virtual void profileSample();
#endif
        /** Returns this thread's ID number. */
        virtual int getThreadNum() { return thread->tid; }

        /** Returns the instruction this thread is currently committing.
         *  Only used when an instruction faults.
         */
        virtual TheISA::MachInst getInst();

        /** Copies the architectural registers from another XC into this XC. */
        virtual void copyArchRegs(ExecContext *xc);

        /** Resets all architectural registers to 0. */
        virtual void clearArchRegs();

        /** Reads an integer register. */
        virtual uint64_t readIntReg(int reg_idx);

        /** Reads a single precision floating point register. */
        virtual float readFloatRegSingle(int reg_idx);

        /** Reads a double precision floating point register. */
        virtual double readFloatRegDouble(int reg_idx);

        /** Reads a floating point register as an integer value. */
        virtual uint64_t readFloatRegInt(int reg_idx);

        /** Sets an integer register to a value. */
        virtual void setIntReg(int reg_idx, uint64_t val);

        /** Sets a single precision fp register to a value. */
        virtual void setFloatRegSingle(int reg_idx, float val);

        /** Sets a double precision fp register to a value. */
        virtual void setFloatRegDouble(int reg_idx, double val);

        /** Sets a fp register to an integer value. */
        virtual void setFloatRegInt(int reg_idx, uint64_t val);

        /** Reads this thread's PC. */
        virtual uint64_t readPC()
        { return cpu->readPC(thread->tid); }

        /** Sets this thread's PC. */
        virtual void setPC(uint64_t val);

        /** Reads this thread's next PC. */
        virtual uint64_t readNextPC()
        { return cpu->readNextPC(thread->tid); }

        /** Sets this thread's next PC. */
        virtual void setNextPC(uint64_t val);

        /** Reads a miscellaneous register. */
        virtual MiscReg readMiscReg(int misc_reg)
        { return cpu->readMiscReg(misc_reg, thread->tid); }

        /** Reads a misc. register, including any side-effects the
         * read might have as defined by the architecture. */
        virtual MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
        { return cpu->readMiscRegWithEffect(misc_reg, fault, thread->tid); }

        /** Sets a misc. register. */
        virtual Fault setMiscReg(int misc_reg, const MiscReg &val);

        /** Sets a misc. register, including any side-effects the
         * write might have as defined by the architecture. */
        virtual Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val);

        /** Returns the number of consecutive store conditional failures. */
        // @todo: Figure out where these store cond failures should go.
        virtual unsigned readStCondFailures()
        { return thread->storeCondFailures; }

        /** Sets the number of consecutive store conditional failures. */
        virtual void setStCondFailures(unsigned sc_failures)
        { thread->storeCondFailures = sc_failures; }

#if FULL_SYSTEM
        /** Returns if the thread is currently in PAL mode, based on
         * the PC's value. */
        virtual bool inPalMode()
        { return TheISA::PcPAL(cpu->readPC(thread->tid)); }
#endif
        // Only really makes sense for old CPU model.  Lots of code
        // outside the CPU still checks this function, so it will
        // always return false to keep everything working.
        /** Checks if the thread is misspeculating.  Because it is
         * very difficult to determine if the thread is
         * misspeculating, this is set as false. */
        virtual bool misspeculating() { return false; }

#if !FULL_SYSTEM
        /** Gets a syscall argument by index. */
        virtual IntReg getSyscallArg(int i);

        /** Sets a syscall argument. */
        virtual void setSyscallArg(int i, IntReg val);

        /** Sets the syscall return value. */
        virtual void setSyscallReturn(SyscallReturn return_value);

        /** Executes a syscall in SE mode. */
        virtual void syscall() { return cpu->syscall(thread->tid); }

        /** Reads the funcExeInst counter. */
        virtual Counter readFuncExeInst() { return thread->funcExeInst; }
#endif
    };

#if FULL_SYSTEM
    /** ITB pointer. */
    AlphaITB *itb;
    /** DTB pointer. */
    AlphaDTB *dtb;
#endif

    /** Registers statistics. */
    void regStats();

#if FULL_SYSTEM
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
    /** Reads a miscellaneous register. */
    MiscReg readMiscReg(int misc_reg, unsigned tid);

    /** Reads a misc. register, including any side effects the read
     * might have as defined by the architecture.
     */
    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault, unsigned tid);

    /** Sets a miscellaneous register. */
    Fault setMiscReg(int misc_reg, const MiscReg &val, unsigned tid);

    /** Sets a misc. register, including any side effects the write
     * might have as defined by the architecture.
     */
    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val, unsigned tid);

    /** Initiates a squash of all in-flight instructions for a given
     * thread.  The source of the squash is an external update of
     * state through the XC.
     */
    void squashFromXC(unsigned tid);

#if FULL_SYSTEM
    /** Posts an interrupt. */
    void post_interrupt(int int_num, int index);
    /** Reads the interrupt flag. */
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
    bool simPalCheck(int palFunc, unsigned tid);

    /** Processes any interrupts. */
    void processInterrupts();

    /** Halts the CPU. */
    void halt() { panic("Halt not implemented!\n"); }
#endif


#if !FULL_SYSTEM
    /** Executes a syscall.
     * @todo: Determine if this needs to be virtual.
     */
    void syscall(int tid);
    /** Gets a syscall argument. */
    IntReg getSyscallArg(int i, int tid);

    /** Used to shift args for indirect syscall. */
    void setSyscallArg(int i, IntReg val, int tid);

    /** Sets the return value of a syscall. */
    void setSyscallReturn(SyscallReturn return_value, int tid);
#endif

    /** Read from memory function. */
    template <class T>
    Fault read(MemReqPtr &req, T &data)
    {
#if 0
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        if (req->flags & LOCKED) {
            req->xc->setMiscReg(TheISA::Lock_Addr_DepTag, req->paddr);
            req->xc->setMiscReg(TheISA::Lock_Flag_DepTag, true);
        }
#endif
#endif
        Fault error;

#if FULL_SYSTEM
        // @todo: Fix this LL/SC hack.
        if (req->flags & LOCKED) {
            lockAddr = req->paddr;
            lockFlag = true;
        }
#endif

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

    /** Write to memory function. */
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

#if FULL_SYSTEM
        // @todo: Fix this LL/SC hack.
        if (req->flags & LOCKED) {
            if (req->flags & UNCACHEABLE) {
                req->result = 2;
            } else {
                if (this->lockFlag) {
                    req->result = 1;
                } else {
                    req->result = 0;
                    return NoFault;
                }
            }
        }
#endif

        return this->mem->write(req, (T)htog(data));
    }

    /** CPU write function, forwards write to LSQ. */
    template <class T>
    Fault write(MemReqPtr &req, T &data, int store_idx)
    {
        return this->iew.ldstQueue.write(req, data, store_idx);
    }

    Addr lockAddr;

    /** Temporary fix for the lock flag, works in the UP case. */
    bool lockFlag;
};

#endif // __CPU_O3_ALPHA_FULL_CPU_HH__
