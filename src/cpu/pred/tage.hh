/*
 * Copyright (c) 2022-2023 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2014 The University of Wisconsin
 *
 * Copyright (c) 2006 INRIA (Institut National de Recherche en
 * Informatique et en Automatique  / French National Research Institute
 * for Computer Science and Applied Mathematics)
 *
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

/* @file
 * Implementation of a TAGE branch predictor. TAGE is a global-history based
 * branch predictor. It features a PC-indexed bimodal predictor and N
 * partially tagged tables, indexed with a hash of the PC and the global
 * branch history. The different lengths of global branch history used to
 * index the partially tagged tables grow geometrically. A small path history
 * is also used in the hash.
 *
 * All TAGE tables are accessed in parallel, and the one using the longest
 * history that matches provides the prediction (some exceptions apply).
 * Entries are allocated in components using a longer history than the
 * one that predicted when the prediction is incorrect.
 */

#ifndef __CPU_PRED_TAGE_HH__
#define __CPU_PRED_TAGE_HH__

#include <vector>

#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/tage_base.hh"
#include "params/TAGE.hh"

namespace gem5
{

namespace branch_prediction
{

class TAGE: public BPredUnit
{
  protected:
    TAGEBase *tage;

    struct TageBranchInfo
    {
        TAGEBase::BranchInfo *tageBranchInfo;

        TageBranchInfo(TAGEBase &tage) : tageBranchInfo(tage.makeBranchInfo())
        {}

        virtual ~TageBranchInfo()
        {
            delete tageBranchInfo;
        }
    };

    virtual bool predict(ThreadID tid, Addr branch_pc, bool cond_branch,
                         void* &b);

  public:

    TAGE(const TAGEParams &params);

    // Base class methods.
    bool lookup(ThreadID tid, Addr branch_addr, void* &bpHistory) override;
    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target,  void * &bpHistory) override;
    void update(ThreadID tid, Addr branch_addr, bool taken, void * &bpHistory,
                bool squashed, const StaticInstPtr & inst,
                Addr corrTarget) override;
    virtual void squash(ThreadID tid, void * &bpHistory) override;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_TAGE_HH__
