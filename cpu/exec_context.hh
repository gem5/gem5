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

#ifndef __CPU_EXEC_CONTEXT_HH__
#define __CPU_EXEC_CONTEXT_HH__

#include "config/full_system.hh"
#include "mem/functional/functional.hh"
#include "mem/mem_req.hh"
#include "sim/eventq.hh"
#include "sim/host.hh"
#include "sim/serialize.hh"
#include "arch/isa_traits.hh"
//#include "arch/isa_registers.hh"
#include "sim/byteswap.hh"

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
namespace Kernel { class Binning; class Statistics; }

#else // !FULL_SYSTEM

#include "sim/process.hh"

#endif // FULL_SYSTEM

//
// The ExecContext object represents a functional context for
// instruction execution.  It incorporates everything required for
// architecture-level functional simulation of a single thread.
//

class ExecContext
{
  protected:
    typedef TheISA::RegFile RegFile;
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscRegFile MiscRegFile;
    typedef TheISA::MiscReg MiscReg;
  public:
    enum Status
    {
        /// Initialized but not running yet.  All CPUs start in
        /// this state, but most transition to Active on cycle 1.
        /// In MP or SMT systems, non-primary contexts will stay
        /// in this state until a thread is assigned to them.
        Unallocated,

        /// Running.  Instructions should be executed only when
        /// the context is in this state.
        Active,

        /// Temporarily inactive.  Entered while waiting for
        /// initialization,synchronization, etc.
        Suspended,

        /// Permanently shut down.  Entered when target executes
        /// m5exit pseudo-instruction.  When all contexts enter
        /// this state, the simulation will terminate.
        Halted
    };

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

  public:
    RegFile regs;	// correct-path register context

    // pointer to CPU associated with this context
    BaseCPU *cpu;

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

    Kernel::Binning *kernelBinning;
    Kernel::Statistics *kernelStats;
    bool bin;
    bool fnbin;

    FunctionProfile *profile;
    ProfileNode *profileNode;
    Addr profilePC;
    void dumpFuncProfile();

    /** Event for timing out quiesce instruction */
    struct EndQuiesceEvent : public Event
    {
        /** A pointer to the execution context that is quiesced */
        ExecContext *xc;

        EndQuiesceEvent(ExecContext *_xc);

        /** Event process to occur at interrupt*/
        virtual void process();

        /** Event description */
        virtual const char *description();
    };
    EndQuiesceEvent quiesceEvent;

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
    ExecContext(BaseCPU *_cpu, int _thread_num, System *_system,
                AlphaITB *_itb, AlphaDTB *_dtb, FunctionalMemory *_dem);
#else
    ExecContext(BaseCPU *_cpu, int _thread_num, Process *_process, int _asid);
    ExecContext(BaseCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                int _asid);
#endif
    virtual ~ExecContext();

    virtual void takeOverFrom(ExecContext *oldContext);

    void regStats(const std::string &name);

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);

#if FULL_SYSTEM
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
            MiscRegFile *cregs = &req->xc->regs.miscRegs;
            cregs->setReg(TheISA::Lock_Addr_DepTag, req->paddr);
            cregs->setReg(TheISA::Lock_Flag_DepTag, true);
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

        MiscRegFile *cregs;

        // If this is a store conditional, act appropriately
        if (req->flags & LOCKED) {
            cregs = &req->xc->regs.miscRegs;

            if (req->flags & UNCACHEABLE) {
                // Don't update result register (see stq_c in isa_desc)
                req->result = 2;
                req->xc->storeCondFailures = 0;//Needed? [RGD]
            } else {
                bool lock_flag = cregs->readReg(TheISA::Lock_Flag_DepTag);
                Addr lock_addr = cregs->readReg(TheISA::Lock_Addr_DepTag);
                req->result = lock_flag;
                if (!lock_flag ||
                    ((lock_addr & ~0xf) != (req->paddr & ~0xf))) {
                    cregs->setReg(TheISA::Lock_Flag_DepTag, false);
                    if (((++req->xc->storeCondFailures) % 100000) == 0) {
                        std::cerr << "Warning: "
                                  << req->xc->storeCondFailures
                                  << " consecutive store conditional failures "
                                  << "on cpu " << req->xc->cpu_id
                                  << std::endl;
                    }
                    return NoFault;
                }
                else req->xc->storeCondFailures = 0;
            }
        }

        // Need to clear any locked flags on other proccessors for
        // this address.  Only do this for succsful Store Conditionals
        // and all other stores (WH64?).  Unsuccessful Store
        // Conditionals would have returned above, and wouldn't fall
        // through.
        for (int i = 0; i < system->execContexts.size(); i++){
            cregs = &system->execContexts[i]->regs.miscRegs;
            if ((cregs->readReg(TheISA::Lock_Addr_DepTag) & ~0xf) ==
                (req->paddr & ~0xf)) {
                cregs->setReg(TheISA::Lock_Flag_DepTag, false);
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
        return regs.miscRegs.readRegWithEffect(misc_reg, fault, this);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return regs.miscRegs.setRegWithEffect(misc_reg, val, this);
    }

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
        TheISA::setSyscallReturn(return_value, &regs);
    }

    void syscall()
    {
        process->syscall(this);
    }
#endif
};


// for non-speculative execution context, spec_mode is always false
inline bool
ExecContext::misspeculating()
{
    return false;
}

#endif // __CPU_EXEC_CONTEXT_HH__
