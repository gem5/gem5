//Todo:

#ifndef __ALPHA_DYN_INST_HH__
#define __ALPHA_DYN_INST_HH__

#include "cpu/base_dyn_inst.hh"
#include "cpu/beta_cpu/alpha_full_cpu.hh"
#include "cpu/beta_cpu/alpha_impl.hh"
#include "cpu/inst_seq.hh"

using namespace std;

class AlphaDynInst : public BaseDynInst<AlphaSimpleImpl>
{
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

    /** Location of this instruction within the ROB.  Might be somewhat
     *  implementation specific.
     *  Might not want this data in the inst as it may be deleted prior to
     *  execution of the stage that needs it.
     */
    int robIdx;

    int getROBEntry()
    {
        return robIdx;
    }

    void setROBEntry(int rob_idx)
    {
        robIdx = rob_idx;
    }

    /** Location of this instruction within the IQ.  Might be somewhat
     *  implementation specific.
     *  Might not want this data in the inst as it may be deleted prior to
     *  execution of the stage that needs it.
     */
    int iqIdx;

    int getIQEntry()
    {
        return iqIdx;
    }

    void setIQEntry(int iq_idx)
    {
        iqIdx = iq_idx;
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

