#ifndef __ALPHA_DYN_INST_CC__
#define __ALPHA_DYN_INST_CC__

#include "cpu/beta_cpu/alpha_dyn_inst.hh"

// Force instantiation of BaseDynInst
template BaseDynInst<AlphaSimpleImpl>;

AlphaDynInst::AlphaDynInst(MachInst inst, Addr PC, Addr Pred_PC,
                           InstSeqNum seq_num, FullCPU *cpu)
    : BaseDynInst<AlphaSimpleImpl>(inst, PC, Pred_PC, seq_num, cpu)
{
    // Initialize these to illegal values.
    robIdx = -1;
    iqIdx = -1;
}

AlphaDynInst::AlphaDynInst(StaticInstPtr<AlphaISA> &_staticInst)
    : BaseDynInst<AlphaSimpleImpl>(_staticInst)
{
}

uint64_t
AlphaDynInst::readUniq()
{
    return cpu->readUniq();
}

void
AlphaDynInst::setUniq(uint64_t val)
{
    cpu->setUniq(val);
}

uint64_t
AlphaDynInst::readFpcr()
{
    return cpu->readFpcr();
}

void
AlphaDynInst::setFpcr(uint64_t val)
{
    cpu->setFpcr(val);
}

#ifdef FULL_SYSTEM
uint64_t
AlphaDynInst::readIpr(int idx, Fault &fault)
{
    return cpu->readIpr(idx, fault);
}
Fault
AlphaDynInst::setIpr(int idx, uint64_t val)
{
    return cpu->setIpr(idx, val);
}

Fault
AlphaDynInst::hwrei()
{
    return cpu->hwrei();
}

int
AlphaDynInst::readIntrFlag()
{
return cpu->readIntrFlag();
}

void
AlphaDynInst::setIntrFlag(int val)
{
    cpu->setIntrFlag(val);
}

bool
AlphaDynInst::inPalMode()
{
    return cpu->inPalMode();
}

void
AlphaDynInst::trap(Fault fault)
{
    cpu->trap(fault);
}

bool
AlphaDynInst::simPalCheck(int palFunc)
{
    return cpu->simPalCheck(palFunc);
}
#else
void
AlphaDynInst::syscall()
{
    cpu->syscall();
}
#endif

#endif // __ALPHA_DYN_INST_CC__
