//Todo:

#ifndef __CPU_BETA_CPU_ALPHA_DYN_INST_HH__
#define __CPU_BETA_CPU_ALPHA_DYN_INST_HH__

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
    /** Typedef for the CPU. */
    typedef typename Impl::FullCPU FullCPU;

    /** Typedef to get the ISA. */
    typedef typename Impl::ISA ISA;

    /** Binary machine instruction type. */
    typedef typename ISA::MachInst MachInst;
    /** Memory address type. */
    typedef typename ISA::Addr	   Addr;
    /** Logical register index type. */
    typedef typename ISA::RegIndex RegIndex;
    /** Integer register index type. */
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

    /** Executes the instruction.  Why the hell did I put this here? */
    Fault execute()
    {
        this->fault = this->staticInst->execute(this, this->traceData);
        return this->fault;
    }

  public:
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



  private:
    /** Physical register index of the destination registers of this
     *  instruction.
     */
    PhysRegIndex _destRegIdx[MaxInstDestRegs];

    /** Physical register index of the source registers of this
     *  instruction.
     */
    PhysRegIndex _srcRegIdx[MaxInstSrcRegs];

    /** Physical register index of the previous producers of the
     *  architected destinations.
     */
    PhysRegIndex _prevDestRegIdx[MaxInstDestRegs];

  public:

    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to redice overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

    uint64_t readIntReg(StaticInst<ISA> *si, int idx)
    {
        return this->cpu->readIntReg(_srcRegIdx[idx]);
    }

    float readFloatRegSingle(StaticInst<ISA> *si, int idx)
    {
        return this->cpu->readFloatRegSingle(_srcRegIdx[idx]);
    }

    double readFloatRegDouble(StaticInst<ISA> *si, int idx)
    {
        return this->cpu->readFloatRegDouble(_srcRegIdx[idx]);
    }

    uint64_t readFloatRegInt(StaticInst<ISA> *si, int idx)
    {
        return this->cpu->readFloatRegInt(_srcRegIdx[idx]);
    }
    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void setIntReg(StaticInst<ISA> *si, int idx, uint64_t val)
    {
        this->cpu->setIntReg(_destRegIdx[idx], val);
        this->instResult.integer = val;
    }

    void setFloatRegSingle(StaticInst<ISA> *si, int idx, float val)
    {
        this->cpu->setFloatRegSingle(_destRegIdx[idx], val);
        this->instResult.fp = val;
    }

    void setFloatRegDouble(StaticInst<ISA> *si, int idx, double val)
    {
        this->cpu->setFloatRegDouble(_destRegIdx[idx], val);
        this->instResult.dbl = val;
    }

    void setFloatRegInt(StaticInst<ISA> *si, int idx, uint64_t val)
    {
        this->cpu->setFloatRegInt(_destRegIdx[idx], val);
        this->instResult.integer = val;
    }

    /** Returns the physical register index of the i'th destination
     *  register.
     */
    PhysRegIndex renamedDestRegIdx(int idx) const
    {
        return _destRegIdx[idx];
    }

    /** Returns the physical register index of the i'th source register. */
    PhysRegIndex renamedSrcRegIdx(int idx) const
    {
        return _srcRegIdx[idx];
    }

    /** Returns the physical register index of the previous physical register
     *  that remapped to the same logical register index.
     */
    PhysRegIndex prevDestRegIdx(int idx) const
    {
        return _prevDestRegIdx[idx];
    }

    /** Renames a destination register to a physical register.  Also records
     *  the previous physical register that the logical register mapped to.
     */
    void renameDestReg(int idx,
                       PhysRegIndex renamed_dest,
                       PhysRegIndex previous_rename)
    {
        _destRegIdx[idx] = renamed_dest;
        _prevDestRegIdx[idx] = previous_rename;
    }

    /** Renames a source logical register to the physical register which
     *  has/will produce that logical register's result.
     *  @todo: add in whether or not the source register is ready.
     */
    void renameSrcReg(int idx, PhysRegIndex renamed_src)
    {
        _srcRegIdx[idx] = renamed_src;
    }

  public:
    Fault calcEA()
    {
        return this->staticInst->eaCompInst()->execute(this, this->traceData);
    }

    Fault memAccess()
    {
        return this->staticInst->memAccInst()->execute(this, this->traceData);
    }
};

#endif // __CPU_BETA_CPU_ALPHA_DYN_INST_HH__

