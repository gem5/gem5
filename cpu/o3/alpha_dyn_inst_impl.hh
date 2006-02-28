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

#include "cpu/o3/alpha_dyn_inst.hh"

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
AlphaDynInst<Impl>::AlphaDynInst(StaticInstPtr &_staticInst)
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

#if FULL_SYSTEM
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

