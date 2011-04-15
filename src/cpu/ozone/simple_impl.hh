/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_OZONE_SIMPLE_IMPL_HH__
#define __CPU_OZONE_SIMPLE_IMPL_HH__

#include "cpu/o3/bpred_unit.hh"
#include "cpu/ozone/cpu.hh"
#include "cpu/ozone/dyn_inst.hh"
#include "cpu/ozone/front_end.hh"
#include "cpu/ozone/inorder_back_end.hh"
#include "cpu/ozone/null_predictor.hh"
#include "cpu/ozone/simple_params.hh"

//template <class Impl>
//class OzoneCPU;

template <class Impl>
class OzoneDynInst;

struct SimpleImpl {
    typedef SimpleParams Params;
    typedef OzoneCPU<SimpleImpl> OzoneCPU;
    typedef OzoneCPU FullCPU;

    // Would like to put these into their own area.
//    typedef NullPredictor BranchPred;
    typedef BPredUnit<SimpleImpl> BranchPred;
    typedef FrontEnd<SimpleImpl> FrontEnd;
    // Will need IQ, LSQ eventually
    typedef InorderBackEnd<SimpleImpl> BackEnd;

    typedef OzoneDynInst<SimpleImpl> DynInst;
    typedef RefCountingPtr<DynInst> DynInstPtr;

    typedef uint64_t IssueStruct;

    enum {
        MaxThreads = 1
    };
};

#endif // __CPU_OZONE_SIMPLE_IMPL_HH__
