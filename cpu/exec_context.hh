/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __EXEC_CONTEXT_HH__
#define __EXEC_CONTEXT_HH__

#include "sim/host.hh"
#include "targetarch/mem_req.hh"

// forward declaration: see functional_memory.hh
class FunctionalMemory;
class PhysicalMemory;
class BaseCPU;

#ifdef FULL_SYSTEM

#include "targetarch/alpha_memory.hh"
class MemoryController;

#include "kern/tru64/kernel_stats.hh"
#include "sim/system.hh"

#else // !FULL_SYSTEM

#include "sim/prog.hh"

#endif // FULL_SYSTEM

//
// The ExecContext object represents a functional context for
// instruction execution.  It incorporates everything required for
// architecture-level functional simulation of a single thread.
//

class ExecContext
{
  public:
    enum Status { Unallocated, Active, Suspended, Halted };

  private:
    Status _status;

  public:
    Status status() const { return _status; }
    void setStatus(Status new_status);

#ifdef FULL_SYSTEM
  public:
    KernelStats kernelStats;
#endif

  public:
    RegFile regs;	// correct-path register context

    // pointer to CPU associated with this context
    BaseCPU *cpu;

    // Index of hardware thread context on the CPU that this represents.
    int thread_num;

#ifdef FULL_SYSTEM

    FunctionalMemory *mem;
    AlphaItb *itb;
    AlphaDtb *dtb;
    int cpu_id;
    System *system;

    // the following two fields are redundant, since we can always
    // look them up through the system pointer, but we'll leave them
    // here for now for convenience
    MemoryController *memCtrl;
    PhysicalMemory *physmem;

#else
    Process *process;

    FunctionalMemory *mem;	// functional storage for process address space

    // Address space ID.  Note that this is used for TIMING cache
    // simulation only; all functional memory accesses should use
    // one of the FunctionalMemory pointers above.
    short asid;

#endif


    /*
     * number of executed instructions, for matching with syscall trace
     * points in EIO files.
     */
    Counter func_exe_insn;

    //
    // Count failed store conditionals so we can warn of apparent
    // application deadlock situations.
    unsigned storeCondFailures;

    // constructor: initialize context from given process structure
#ifdef FULL_SYSTEM
    ExecContext(BaseCPU *_cpu, int _thread_num, System *_system,
                AlphaItb *_itb, AlphaDtb *_dtb, FunctionalMemory *_dem,
                int _cpu_id);
#else
    ExecContext(BaseCPU *_cpu, int _thread_num, Process *_process, int _asid);
    ExecContext(BaseCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                int _asid);
#endif
    virtual ~ExecContext() {}

    void regStats(const std::string &name);

#ifdef FULL_SYSTEM
    bool validInstAddr(Addr addr) { return true; }
    bool validDataAddr(Addr addr) { return true; }
    int getInstAsid() { return ITB_ASN_ASN(regs.ipr[TheISA::IPR_ITB_ASN]); }
    int getDataAsid() { return DTB_ASN_ASN(regs.ipr[TheISA::IPR_DTB_ASN]); }

    Fault translateInstReq(MemReqPtr req)
    {
        return itb->translate(req);
    }

    Fault translateDataReadReq(MemReqPtr req)
    {
        return dtb->translate(req, false);
    }

    Fault translateDataWriteReq(MemReqPtr req)
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

    Fault dummyTranslation(MemReqPtr req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return No_Fault;
    }
    Fault translateInstReq(MemReqPtr req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataReadReq(MemReqPtr req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataWriteReq(MemReqPtr req)
    {
        return dummyTranslation(req);
    }

#endif

    template <class T>
    Fault read(MemReqPtr req, T& data)
    {
#if defined(TARGET_ALPHA) && defined(FULL_SYSTEM)
        if (req->flags & LOCKED) {
            MiscRegFile *cregs = &req->xc->regs.miscRegs;
            cregs->lock_addr = req->paddr;
            cregs->lock_flag = true;
        }
#endif
        return mem->read(req, data);
    }

    template <class T>
    Fault write(MemReqPtr req, T& data)
    {
#if defined(TARGET_ALPHA) && defined(FULL_SYSTEM)

        MiscRegFile *cregs;

        // If this is a store conditional, act appropriately
        if (req->flags & LOCKED) {
            cregs = &req->xc->regs.miscRegs;

            if (req->flags & UNCACHEABLE) {
                // Don't update result register (see machine.def)
                req->result = 2;
                req->xc->storeCondFailures = 0;//Needed? [RGD]
            } else {
                req->result = cregs->lock_flag;
                if (!cregs->lock_flag ||
                    ((cregs->lock_addr & ~0xf) != (req->paddr & ~0xf))) {
                    cregs->lock_flag = false;
                    if (((++req->xc->storeCondFailures) % 100000) == 0) {
                        std::cerr << "Warning: "
                                  << req->xc->storeCondFailures
                                  << " consecutive store conditional failures "
                                  << "on cpu " << req->xc->cpu_id
                                  << std::endl;
                    }
                    return No_Fault;
                }
                else req->xc->storeCondFailures = 0;
            }
        }

        // Need to clear any locked flags on other proccessors for this
        // address
        // Only do this for succsful Store Conditionals and all other
        // stores (WH64?)
        // Unsuccesful Store Conditionals would have returned above,
        // and wouldn't fall through
        for (int i = 0; i < system->xcvec.size(); i++){
            cregs = &system->xcvec[i]->regs.miscRegs;
            if ((cregs->lock_addr & ~0xf) == (req->paddr & ~0xf)) {
                cregs->lock_flag = false;
            }
        }

#endif
        return mem->write(req, data);
    }

    virtual bool misspeculating();


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

    uint64_t readUniq()
    {
        return regs.miscRegs.uniq;
    }

    void setUniq(uint64_t val)
    {
        regs.miscRegs.uniq = val;
    }

    uint64_t readFpcr()
    {
        return regs.miscRegs.fpcr;
    }

    void setFpcr(uint64_t val)
    {
        regs.miscRegs.fpcr = val;
    }

#ifdef FULL_SYSTEM
    uint64_t readIpr(int idx, Fault &fault);
    Fault setIpr(int idx, uint64_t val);
    Fault hwrei();
    void ev5_trap(Fault fault);
    bool simPalCheck(int palFunc);
#endif

#ifndef FULL_SYSTEM
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

#endif // __EXEC_CONTEXT_HH__
