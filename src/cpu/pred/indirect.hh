/*
 * Copyright (c) 2014 ARM Limited
 * Copyright (c) 2023 The University of Edinburgh
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
 * Indirect target predictor interface
 */

#ifndef __CPU_PRED_INDIRECT_BASE_HH__
#define __CPU_PRED_INDIRECT_BASE_HH__

#include "arch/generic/pcstate.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/branch_type.hh"
#include "params/IndirectPredictor.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace branch_prediction
{

class IndirectPredictor : public SimObject
{
  public:
    typedef IndirectPredictorParams Params;

    IndirectPredictor(const Params &params) : SimObject(params) {}

    virtual void reset(){};

    /**
     * Predicts the indirect target of an indirect branch.
     * @param tid Thread ID of the branch.
     * @param sn The sequence number of the branch.
     * @param pc The branch PC address.
     * @param i_history The pointer to the history object.
     * @return For a hit the predictor returns a pointer to the target PCState
     *         otherwise a nullptr is returned.
     */
    virtual const PCStateBase *lookup(ThreadID tid, InstSeqNum sn, Addr pc,
                                      void *&i_history) = 0;

    /**
     * Updates the indirect predictor with history information of a branch.
     * Is called right after the prediction which updates the state
     * speculatively. In case the branch was mispredicted the function
     * is called again with the corrected information.
     * The function is called for ALL branches as some predictors incooperate
     * all branches in their history.
     * @param tid Thread ID
     * @param sn The sequence number of the branch.
     * @param pc The branch PC address.
     * @param squash Whether the update is called at a misprediction
     * @param taken Whether a conditional branch was taken
     * @param target The target address if this branch.
     * @param br_type The branch instruction type.
     * @param i_history The pointer to the history object.
     */
    virtual void update(ThreadID tid, InstSeqNum sn, Addr pc, bool squash,
                        bool taken, const PCStateBase &target,
                        BranchType br_type, void *&i_history) = 0;

    /**
     * Squashes a branch. If the branch modified the history
     * reverts the modification.
     * @param tid Thread ID
     * @param sn The sequence number of the branch.
     * @param i_history The pointer to the history object.
     */
    virtual void squash(ThreadID tid, InstSeqNum sn, void *&i_history) = 0;

    /**
     * A branch gets finally commited. Updates the internal state of
     * the indirect predictor (counter and target information).
     * @param tid Thread ID
     * @param sn The sequence number of the branch.
     * @param i_history The pointer to the history object.
     */
    virtual void commit(ThreadID tid, InstSeqNum sn, void *&i_history) = 0;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_INDIRECT_BASE_HH__
