//Todo:

#ifndef __ALPHA_DYN_INST_HH__
#define __ALPHA_DYN_INST_HH__

#include "cpu/base_dyn_inst.hh"
#include "cpu/beta_cpu/alpha_full_cpu.hh"
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/inst_seq.hh"

/**
 * Mostly implementation specific AlphaDynInst.  It is templated in case there
 * are other implementations that are similar enough to be able to use this
 * class without changes.  This is mainly useful if there are multiple similar
 * CPU implementations of the same ISA.
 */

template <class Impl>
class AlphaDynInst : public BaseDynInst<Impl>
{
  public:
    // Typedef for the CPU.
    typedef typename Impl::FullCPU FullCPU;

    //Typedef to get the ISA.
    typedef typename Impl::ISA ISA;

    /// Binary machine instruction type.
    typedef typename ISA::MachInst MachInst;
    /// Memory address type.
    typedef typename ISA::Addr	   Addr;
    /// Logical register index type.
    typedef typename ISA::RegIndex RegIndex;
    /// Integer register index type.
    typedef typename ISA::IntReg   IntReg;

    enum {
        MaxInstSrcRegs = ISA::MaxInstSrcRegs,	//< Max source regs
        MaxInstDestRegs = ISA::MaxInstDestRegs,	//< Max dest regs
    };

  public:
    /** BaseDynInst constructor given a binary instruction. */
    AlphaDynInst(MachInst inst, Addr PC, Addr Pred_PC, InstSeqNum seq_num,
                 FullCPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    AlphaDynInst(StaticInstPtr<AlphaISA> &_staticInst);

    /** Executes the instruction. */
    Fault execute()
    {
        fault = staticInst->execute(this, traceData);
        return fault;
    }

    uint64_t readUniq();
    void setUniq(uint64_t val);

    uint64_t readFpcr();
    void setFpcr(uint64_t val);

#ifdef FULL_SYSTEM
    uint64_t readIpr(int idx, Fault &fault);
    Fault setIpr(int idx, uint64_t val);
    Fault hwrei();
    int readIntrFlag();
    void setIntrFlag(int val);
    bool inPalMode();
    void trap(Fault fault);
    bool simPalCheck(int palFunc);
#else
    void syscall();
#endif

};

#endif // __ALPHA_DYN_INST_HH__

