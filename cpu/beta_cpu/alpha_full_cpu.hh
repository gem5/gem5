// Todo: Find all the stuff in ExecContext and ev5 that needs to be
// specifically designed for this CPU.
// Read and write are horribly hacked up between not being sure where to
// copy their code from, and Ron's memory changes.

#ifndef __CPU_BETA_CPU_ALPHA_FULL_CPU_HH__
#define __CPU_BETA_CPU_ALPHA_FULL_CPU_HH__

// To include: comm, full cpu, ITB/DTB if full sys,
#include "cpu/beta_cpu/full_cpu.hh"

template <class Impl>
class AlphaFullCPU : public FullBetaCPU<Impl>
{
  public:
    typedef typename Impl::ISA AlphaISA;
    typedef typename Impl::Params Params;

  public:
    AlphaFullCPU(Params &params);

#ifdef FULL_SYSTEM
    AlphaITB *itb;
    AlphaDTB *dtb;
#endif

  public:
    void regStats();

#ifdef FULL_SYSTEM
    bool inPalMode();

    //Note that the interrupt stuff from the base CPU might be somewhat
    //ISA specific (ie NumInterruptLevels).  These functions might not
    //be needed in FullCPU though.
//    void post_interrupt(int int_num, int index);
//    void clear_interrupt(int int_num, int index);
//    void clear_interrupts();

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
    Fault dummyTranslation(MemReqPtr &req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return No_Fault;
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

    // Later on may want to remove this misc stuff from the regfile and
    // have it handled at this level.  Might prove to be an issue when
    // trying to rename source/destination registers...
    uint64_t readUniq()
    {
        return regFile.readUniq();
    }

    void setUniq(uint64_t val)
    {
        regFile.setUniq(val);
    }

    uint64_t readFpcr()
    {
        return regFile.readFpcr();
    }

    void setFpcr(uint64_t val)
    {
        regFile.setFpcr(val);
    }

#ifdef FULL_SYSTEM
    uint64_t *getIPR();
    uint64_t readIpr(int idx, Fault &fault);
    Fault setIpr(int idx, uint64_t val);
    int readIntrFlag();
    void setIntrFlag(int val);
    Fault hwrei();
    bool inPalMode();
    void trap(Fault fault);
    bool simPalCheck(int palFunc);

    void processInterrupts();
#endif


#ifndef FULL_SYSTEM
    // Need to change these into regfile calls that directly set a certain
    // register.  Actually, these functions should handle most of this
    // functionality by themselves; should look up the rename and then
    // set the register.
    IntReg getSyscallArg(int i)
    {
        return xc->regs.intRegFile[AlphaISA::ArgumentReg0 + i];
    }

    // used to shift args for indirect syscall
    void setSyscallArg(int i, IntReg val)
    {
        xc->regs.intRegFile[AlphaISA::ArgumentReg0 + i] = val;
    }

    void setSyscallReturn(int64_t return_value)
    {
        // check for error condition.  Alpha syscall convention is to
        // indicate success/failure in reg a3 (r19) and put the
        // return value itself in the standard return value reg (v0).
        const int RegA3 = 19;	// only place this is used
        if (return_value >= 0) {
            // no error
            xc->regs.intRegFile[RegA3] = 0;
            xc->regs.intRegFile[AlphaISA::ReturnValueReg] = return_value;
        } else {
            // got an error, return details
            xc->regs.intRegFile[RegA3] = (IntReg) -1;
            xc->regs.intRegFile[AlphaISA::ReturnValueReg] = -return_value;
        }
    }

    void syscall();
    void squashStages();

#endif

    void copyToXC();
    void copyFromXC();

  public:
#ifdef FULL_SYSTEM
    bool palShadowEnabled;

    // Not sure this is used anywhere.
    void intr_post(RegFile *regs, Fault fault, Addr pc);
    // Actually used within exec files.  Implement properly.
    void swap_palshadow(RegFile *regs, bool use_shadow);
    // Called by CPU constructor.  Can implement as I please.
    void initCPU(RegFile *regs);
    // Called by initCPU.  Implement as I please.
    void initIPRs(RegFile *regs);
#endif


    template <class T>
    Fault read(MemReqPtr &req, T &data)
    {
#if defined(TARGET_ALPHA) && defined(FULL_SYSTEM)
        if (req->flags & LOCKED) {
            MiscRegFile *cregs = &req->xc->regs.miscRegs;
            cregs->lock_addr = req->paddr;
            cregs->lock_flag = true;
        }
#endif

        Fault error;
        error = mem->read(req, data);
        data = htoa(data);
        return error;
    }


    template <class T>
    Fault write(MemReqPtr &req, T &data)
    {
#if defined(TARGET_ALPHA) && defined(FULL_SYSTEM)

        MiscRegFile *cregs;

        // If this is a store conditional, act appropriately
        if (req->flags & LOCKED) {
            cregs = &xc->regs.miscRegs;

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
                                  << "on cpu " << cpu_id
                                  << std::endl;
                    }
                    return No_Fault;
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
            if ((cregs->lock_addr & ~0xf) == (req->paddr & ~0xf)) {
                cregs->lock_flag = false;
            }
        }

#endif

        return mem->write(req, (T)htoa(data));
    }

};

#endif // __CPU_BETA_CPU_ALPHA_FULL_CPU_HH__
