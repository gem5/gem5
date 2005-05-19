
#include "cpu/beta_cpu/alpha_dyn_inst.hh"

template <class Impl>
AlphaDynInst<Impl>::AlphaDynInst(MachInst inst, Addr PC, Addr Pred_PC,
                                 InstSeqNum seq_num, FullCPU *cpu)
    : BaseDynInst<Impl>(inst, PC, Pred_PC, seq_num, cpu)
{
    // Make sure to have the renamed register entries set to the same
    // as the normal register entries.  It will allow the IQ to work
    // without any modifications.
    for (int i = 0; i < this->staticInst->numDestRegs(); i++)
    {
        _destRegIdx[i] = this->staticInst->destRegIdx(i);
    }

    for (int i = 0; i < this->staticInst->numSrcRegs(); i++)
    {
        _srcRegIdx[i] = this->staticInst->srcRegIdx(i);
        this->_readySrcRegIdx[i] = 0;
    }

}

template <class Impl>
AlphaDynInst<Impl>::AlphaDynInst(StaticInstPtr<AlphaISA> &_staticInst)
    : BaseDynInst<Impl>(_staticInst)
{
    // Make sure to have the renamed register entries set to the same
    // as the normal register entries.  It will allow the IQ to work
    // without any modifications.
    for (int i = 0; i < _staticInst->numDestRegs(); i++)
    {
        _destRegIdx[i] = _staticInst->destRegIdx(i);
    }

    for (int i = 0; i < _staticInst->numSrcRegs(); i++)
    {
        _srcRegIdx[i] = _staticInst->srcRegIdx(i);
    }
}

template <class Impl>
uint64_t
AlphaDynInst<Impl>::readUniq()
{
    return this->cpu->readUniq();
}

template <class Impl>
void
AlphaDynInst<Impl>::setUniq(uint64_t val)
{
    this->cpu->setUniq(val);
}

template <class Impl>
uint64_t
AlphaDynInst<Impl>::readFpcr()
{
    return this->cpu->readFpcr();
}

template <class Impl>
void
AlphaDynInst<Impl>::setFpcr(uint64_t val)
{
    this->cpu->setFpcr(val);
}

#ifdef FULL_SYSTEM
template <class Impl>
uint64_t
AlphaDynInst<Impl>::readIpr(int idx, Fault &fault)
{
    return this->cpu->readIpr(idx, fault);
}

template <class Impl>
Fault
AlphaDynInst<Impl>::setIpr(int idx, uint64_t val)
{
    return this->cpu->setIpr(idx, val);
}

template <class Impl>
Fault
AlphaDynInst<Impl>::hwrei()
{
    return this->cpu->hwrei();
}

template <class Impl>
int
AlphaDynInst<Impl>::readIntrFlag()
{
return this->cpu->readIntrFlag();
}

template <class Impl>
void
AlphaDynInst<Impl>::setIntrFlag(int val)
{
    this->cpu->setIntrFlag(val);
}

template <class Impl>
bool
AlphaDynInst<Impl>::inPalMode()
{
    return this->cpu->inPalMode();
}

template <class Impl>
void
AlphaDynInst<Impl>::trap(Fault fault)
{
    this->cpu->trap(fault);
}

template <class Impl>
bool
AlphaDynInst<Impl>::simPalCheck(int palFunc)
{
    return this->cpu->simPalCheck(palFunc);
}
#else
template <class Impl>
void
AlphaDynInst<Impl>::syscall()
{
    this->cpu->syscall(this->threadNumber);
}
#endif

