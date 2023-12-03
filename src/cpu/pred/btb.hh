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

#ifndef __CPU_PRED_BTB_HH__
#define __CPU_PRED_BTB_HH__

#include "arch/generic/pcstate.hh"
#include "base/statistics.hh"
#include "cpu/pred/branch_type.hh"
#include "cpu/static_inst.hh"
#include "params/BranchTargetBuffer.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

namespace branch_prediction
{

class BranchTargetBuffer : public ClockedObject
{
  public:
    typedef BranchTargetBufferParams Params;
    typedef enums::BranchType BranchType;

    BranchTargetBuffer(const Params &params);

    virtual void memInvalidate() override = 0;

    /** Checks if a branch address is in the BTB. Intended as a quick check
     *  before calling lookup. Does not update statistics.
     *  @param inst_PC The address of the branch to look up.
     *  @return Whether or not the branch exists in the BTB.
     */
    virtual bool valid(ThreadID tid, Addr instPC) = 0;

    /** Looks up an address in the BTB to get the target of the branch.
     *  @param inst_PC The address of the branch to look up.
     *  @param type Optional type of the branch to look up.
     *  @return The target of the branch or nullptr if the branch is not
     *          in the BTB.
     */
    virtual const PCStateBase *
    lookup(ThreadID tid, Addr instPC,
           BranchType type = BranchType::NoBranch) = 0;

    /** Looks up an address in the BTB and return the instruction
     * information if existant. Does not update statistics.
     *  @param inst_PC The address of the branch to look up.
     *  @return Returns the target of the branch.
     */
    virtual const StaticInstPtr getInst(ThreadID tid, Addr instPC) = 0;

    /** Updates the BTB with the target of a branch.
     *  @param inst_pc The address of the branch being updated.
     *  @param target_pc The target address of the branch.
     */
    virtual void update(ThreadID tid, Addr inst_pc,
                        const PCStateBase &target_pc,
                        BranchType type = BranchType::NoBranch,
                        StaticInstPtr inst = nullptr) = 0;

    /** Update BTB statistics
     */
    virtual void
    incorrectTarget(Addr inst_pc, BranchType type = BranchType::NoBranch)
    {
        stats.mispredict[type]++;
    }

  protected:
    /** Number of the threads for which the branch history is maintained. */
    const unsigned numThreads;

    struct BranchTargetBufferStats : public statistics::Group
    {
        BranchTargetBufferStats(statistics::Group *parent);

        statistics::Vector lookups;
        statistics::Vector misses;
        statistics::Vector updates;
        statistics::Vector mispredict;
        statistics::Scalar evictions;

    } stats;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_BTB_HH__
