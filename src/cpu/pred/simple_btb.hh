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

#ifndef __CPU_PRED_SIMPLE_BTB_HH__
#define __CPU_PRED_SIMPLE_BTB_HH__

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/pred/btb.hh"
#include "params/SimpleBTB.hh"

namespace gem5
{

namespace branch_prediction
{

class SimpleBTB : public BranchTargetBuffer
{
  public:
    SimpleBTB(const SimpleBTBParams &params);

    void memInvalidate() override;
    bool valid(ThreadID tid, Addr instPC) override;
    const PCStateBase *lookup(ThreadID tid, Addr instPC,
                           BranchType type = BranchType::NoBranch) override;
    void update(ThreadID tid, Addr instPC, const PCStateBase &target_pc,
                           BranchType type = BranchType::NoBranch,
                           StaticInstPtr inst = nullptr) override;
    const StaticInstPtr getInst(ThreadID tid, Addr instPC) override;


  private:
    struct BTBEntry
    {
        /** The entry's tag. */
        Addr tag = 0;

        /** The entry's target. */
        std::unique_ptr<PCStateBase> target;

        /** The entry's thread id. */
        ThreadID tid;

        /** Whether or not the entry is valid. */
        bool valid = false;

        /** Pointer to the static branch instruction at this address */
        StaticInstPtr inst = nullptr;
    };


    /** Returns the index into the BTB, based on the branch's PC.
     *  @param inst_PC The branch to look up.
     *  @return Returns the index into the BTB.
     */
    inline unsigned getIndex(Addr instPC, ThreadID tid);

    /** Returns the tag bits of a given address.
     *  @param inst_PC The branch's address.
     *  @return Returns the tag bits.
     */
    inline Addr getTag(Addr instPC);

    /** Internal call to find an address in the BTB
     * @param instPC The branch's address.
     * @return Returns a pointer to the BTB entry if found, nullptr otherwise.
    */
    BTBEntry *findEntry(Addr instPC, ThreadID tid);

    /** The actual BTB. */
    std::vector<BTBEntry> btb;

    /** The number of entries in the BTB. */
    unsigned numEntries;

    /** The index mask. */
    unsigned idxMask;

    /** The number of tag bits per entry. */
    unsigned tagBits;

    /** The tag mask. */
    unsigned tagMask;

    /** Number of bits to shift PC when calculating index. */
    unsigned instShiftAmt;

    /** Number of bits to shift PC when calculating tag. */
    unsigned tagShiftAmt;

    /** Log2 NumThreads used for hashing threadid */
    unsigned log2NumThreads;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_SIMPLE_BTB_HH__
