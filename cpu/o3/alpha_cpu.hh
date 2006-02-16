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

// Todo: Find all the stuff in ExecContext and ev5 that needs to be
// specifically designed for this CPU.

#ifndef __CPU_O3_CPU_ALPHA_FULL_CPU_HH__
#define __CPU_O3_CPU_ALPHA_FULL_CPU_HH__

#include "cpu/o3/cpu.hh"
#include "arch/isa_traits.hh"
#include "sim/byteswap.hh"

template <class Impl>
class AlphaFullCPU : public FullO3CPU<Impl>
{
  public:
    typedef typename Impl::ISA AlphaISA;
    typedef typename Impl::Params Params;

  public:
    AlphaFullCPU(Params &params);

#if FULL_SYSTEM
    AlphaITB *itb;
    AlphaDTB *dtb;
#endif

  public:
    void regStats();

#if FULL_SYSTEM
    //Note that the interrupt stuff from the base CPU might be somewhat
    //ISA specific (ie NumInterruptLevels).  These functions might not
    //be needed in FullCPU though.
//    void post_interrupt(int int_num, int index);
//    void clear_interrupt(int int_num, int index);
//    void clear_interrupts();

    Fault * translateInstReq(MemReqPtr &req)
    {
        return itb->translate(req);
    }

    Fault * translateDataReadReq(MemReqPtr &req)
    {
        return dtb->translate(req, false);
    }

    Fault * translateDataWriteReq(MemReqPtr &req)
    {
        return dtb->translate(req, true);
    }

#else
    Fault * dummyTranslation(MemReqPtr &req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return NoFault;
    }

    Fault * translateInstReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    Fault * translateDataReadReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

    Fault * translateDataWriteReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

#endif

    // Later on may want to remove this misc stuff from the regfile and
    // have it handled at this level.  Might prove to be an issue when
    // trying to rename source/destination registers...
    uint64_t readUniq()
    {
        return this->regFile.readUniq();
    }

    void setUniq(uint64_t val)
    {
        this->regFile.setUniq(val);
    }

    uint64_t readFpcr()
    {
        return this->regFile.readFpcr();
    }

    void setFpcr(uint64_t val)
    {
        this->regFile.setFpcr(val);
    }

    // Most of the full system code and syscall emulation is not yet
    // implemented.  These functions do show what the final interface will
    // look like.
#if FULL_SYSTEM
    uint64_t *getIpr();
    uint64_t readIpr(int idx, Fault * &fault);
    Fault * setIpr(int idx, uint64_t val);
    int readIntrFlag();
    void setIntrFlag(int val);
    Fault * hwrei();
    bool inPalMode() { return AlphaISA::PcPAL(this->regFile.readPC()); }
    bool inPalMode(uint64_t PC)
    { return AlphaISA::PcPAL(PC); }

    void trap(Fault * fault);
    bool simPalCheck(int palFunc);

    void processInterrupts();
#endif


#if !FULL_SYSTEM
    // Need to change these into regfile calls that directly set a certain
    // register.  Actually, these functions should handle most of this
    // functionality by themselves; should look up the rename and then
    // set the register.
    IntReg getSyscallArg(int i)
    {
        return this->xc->regs.intRegFile[AlphaISA::ArgumentReg0 + i];
    }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, IntReg val)
    {
        this->xc->regs.intRegFile[AlphaISA::ArgumentReg0 + i] = val;
    }

    void setSyscallReturn(int64_t return_value)
    {
        // check for error condition.  Alpha syscall convention is to
        // indicate success/failure in reg a3 (r19) and put the
        // return value itself in the standard return value reg (v0).
        const int RegA3 = 19;	// only place this is used
        if (return_value >= 0) {
            // no error
            this->xc->regs.intRegFile[RegA3] = 0;
            this->xc->regs.intRegFile[AlphaISA::ReturnValueReg] = return_value;
        } else {
            // got an error, return details
            this->xc->regs.intRegFile[RegA3] = (IntReg) -1;
            this->xc->regs.intRegFile[AlphaISA::ReturnValueReg] = -return_value;
        }
    }

    void syscall(short thread_num);
    void squashStages();

#endif

    void copyToXC();
    void copyFromXC();

  public:
#if FULL_SYSTEM
    bool palShadowEnabled;

    // Not sure this is used anywhere.
    void intr_post(RegFile *regs, Fault * fault, Addr pc);
    // Actually used within exec files.  Implement properly.
    void swapPALShadow(bool use_shadow);
    // Called by CPU constructor.  Can implement as I please.
    void initCPU(RegFile *regs);
    // Called by initCPU.  Implement as I please.
    void initIPRs(RegFile *regs);

    void halt() { panic("Halt not implemented!\n"); }
#endif


    template <class T>
    Fault * read(MemReqPtr &req, T &data)
    {
#if FULL_SYSTEM && defined(TARGET_ALPHA)
        if (req->flags & LOCKED) {
            MiscRegFile *cregs = &req->xc->regs.miscRegs;
            cregs->lock_addr = req->paddr;
            cregs->lock_flag = true;
        }
#endif

        Fault * error;
        error = this->mem->read(req, data);
        data = gtoh(data);
        return error;
    }

    template <class T>
    Fault * read(MemReqPtr &req, T &data, int load_idx)
    {
        return this->iew.ldstQueue.read(req, data, load_idx);
    }

    template <class T>
    Fault * write(MemReqPtr &req, T &data)
    {
#if FULL_SYSTEM && defined(TARGET_ALPHA)

        MiscRegFile *cregs;

        // If this is a store conditional, act appropriately
        if (req->flags & LOCKED) {
            cregs = &this->xc->regs.miscRegs;

            if (req->flags & UNCACHEABLE) {
                // Don't update result register (see stq_c in isa_desc)
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
                                  << "on cpu " << this->cpu_id
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
        for (int i = 0; i < this->system->execContexts.size(); i++){
            cregs = &this->system->execContexts[i]->regs.miscRegs;
            if ((cregs->lock_addr & ~0xf) == (req->paddr & ~0xf)) {
                cregs->lock_flag = false;
            }
        }

#endif

        return this->mem->write(req, (T)::htog(data));
    }

    template <class T>
    Fault * write(MemReqPtr &req, T &data, int store_idx)
    {
        return this->iew.ldstQueue.write(req, data, store_idx);
    }

};

#endif // __CPU_O3_CPU_ALPHA_FULL_CPU_HH__
