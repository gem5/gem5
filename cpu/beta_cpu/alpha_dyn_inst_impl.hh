
#include "cpu/beta_cpu/alpha_dyn_inst.hh"

template <class Impl>
AlphaDynInst<Impl>::AlphaDynInst(MachInst inst, Addr PC, Addr Pred_PC,
                                 InstSeqNum seq_num, FullCPU *cpu)
    : BaseDynInst<AlphaSimpleImpl>(inst, PC, Pred_PC, seq_num, cpu)
{
}

template <class Impl>
AlphaDynInst<Impl>::AlphaDynInst(StaticInstPtr<AlphaISA> &_staticInst)
    : BaseDynInst<AlphaSimpleImpl>(_staticInst)
{
}

template <class Impl>
uint64_t
AlphaDynInst<Impl>::readUniq()
{
    return cpu->readUniq();
}

template <class Impl>
void
AlphaDynInst<Impl>::setUniq(uint64_t val)
{
    cpu->setUniq(val);
}

template <class Impl>
uint64_t
AlphaDynInst<Impl>::readFpcr()
{
    return cpu->readFpcr();
}

template <class Impl>
void
AlphaDynInst<Impl>::setFpcr(uint64_t val)
{
    cpu->setFpcr(val);
}

#ifdef FULL_SYSTEM
template <class Impl>
uint64_t
AlphaDynInst<Impl>::readIpr(int idx, Fault &fault)
{
    return cpu->readIpr(idx, fault);
}

template <class Impl>
Fault
AlphaDynInst<Impl>::setIpr(int idx, uint64_t val)
{
    return cpu->setIpr(idx, val);
}

template <class Impl>
Fault
AlphaDynInst<Impl>::hwrei()
{
    return cpu->hwrei();
}

template <class Impl>
int
AlphaDynInst<Impl>::readIntrFlag()
{
return cpu->readIntrFlag();
}

template <class Impl>
void
AlphaDynInst<Impl>::setIntrFlag(int val)
{
    cpu->setIntrFlag(val);
}

template <class Impl>
bool
AlphaDynInst<Impl>::inPalMode()
{
    return cpu->inPalMode();
}

template <class Impl>
void
AlphaDynInst<Impl>::trap(Fault fault)
{
    cpu->trap(fault);
}

template <class Impl>
bool
AlphaDynInst<Impl>::simPalCheck(int palFunc)
{
    return cpu->simPalCheck(palFunc);
}
#else
template <class Impl>
void
AlphaDynInst<Impl>::syscall()
{
    cpu->syscall();
}
#endif

