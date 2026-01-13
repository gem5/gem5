/*
 * Copyright (c) 2025 Technical University of Munich
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
 * Conditional branch predictor interface
 */

#ifndef __CPU_PRED_CONDITIONAL_BASE_HH__
#define __CPU_PRED_CONDITIONAL_BASE_HH__

#include "arch/generic/pcstate.hh"
#include "cpu/inst_seq.hh"
#include "cpu/pred/branch_type.hh"
#include "params/ConditionalPredictor.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace branch_prediction
{

class ConditionalPredictor : public SimObject
{
  public:

    typedef ConditionalPredictorParams Params;

    ConditionalPredictor(const Params &params);


    /**
     * Looks up a given conditional branch PC of in the BP to see if it
     * is taken or not taken.
     * @param tid The thread id.
     * @param pc The PC to look up.
     * @param bp_history Pointer that will be set to an object that
     * has the branch predictor state associated with the lookup.
     * @return Whether the branch is taken or not taken.
     */
    virtual bool lookup(ThreadID tid, Addr pc, void * &bp_history) = 0;

    /**
     * Ones done with the prediction this function updates the
     * path and global history. All branches call this function
     * including unconditional once.
     * @param tid The thread id.
     * @param pc The branch's pc that will be updated.
     * @param uncond Wheather or not this branch is an unconditional branch.
     * @param taken Whether or not the branch was taken
     * @param target The final target of branch. Some modern
     * predictors use the target in their history.
     * @param inst Static instruction information
     * @param bp_history Pointer that will be set to an object that
     * has the branch predictor state associated with the lookup.
     *
     */
    virtual void updateHistories(ThreadID tid, Addr pc, bool uncond,
                                 bool taken, Addr target,
                                 const StaticInstPtr &inst,
                                 void * &bp_history) = 0;

    /**
     * @param tid The thread id.
     * @param bp_history Pointer to the history object.  The predictor
     * will need to update any state and delete the object.
     */
    virtual void squash(ThreadID tid, void * &bp_history) = 0;


    /**
     * Updates the BP with taken/not taken information.
     * @param tid The thread id.
     * @param pc The branch's PC that will be updated.
     * @param taken Whether the branch was taken or not taken.
     * @param bp_history Pointer to the branch predictor state that is
     * associated with the branch lookup that is being updated.
     * @param squashed Set to true when this function is called during a
     * squash operation.
     * @param inst Static instruction information
     * @param target The resolved target of the branch (only needed
     * for squashed branches)
     * @todo Make this update flexible enough to handle a global predictor.
     */
    virtual void update(ThreadID tid, Addr pc, bool taken,
                        void * &bp_history, bool squashed,
                        const StaticInstPtr &inst, Addr target) = 0;

    /**
     * Special function for the decoupled front-end. In it there can be
     * branches which are not detected by the BPU in the first place as it
     * requires a BTB hit. This function will generate a placeholder for
     * such a branch once it is pre-decoded in the fetch stage. It will
     * only create the branch history object but not update any internal state
     * of the BPU.
     * If the branch turns to be wrong then decode or commit will
     * be able to use the normal squash functionality to correct the branch.
     * Note that not all branch predictors implement this functionality.
     * @param tid The thread id.
     * @param pc The branch's PC.
     * @param uncond Whether or not this branch is an unconditional branch.
     * @param bp_history Pointer that will be set to an branch history object.
     */
    virtual void branchPlaceholder(ThreadID tid, Addr pc,
                                   bool uncond, void * &bp_history);
  protected:

    /** Number of bits to shift instructions by for predictor addresses. */
    const unsigned instShiftAmt;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_CONDITIONAL_BASE_HH__
