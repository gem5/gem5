/*
 * Copyright (c) 2026 The University of Edinburgh
 * Copyright (c) 2026 Technical University of Munich
 * All rights reserved
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

#ifndef __CPU_PRED_MULTI_LEVEL_BTB_HH__
#define __CPU_PRED_MULTI_LEVEL_BTB_HH__

#include <vector>

#include "base/cache/associative_cache.hh"
#include "cpu/pred/btb.hh"
#include "cpu/pred/btb_entry.hh"
#include "params/BTBLevel.hh"
#include "params/MultiLevelBTB.hh"
#include "sim/sim_object.hh"

namespace gem5::branch_prediction
{

class BTBLevel : public SimObject
{
  public:
    BTBLevel(const BTBLevelParams &params);

  private:
    friend class MultiLevelBTB;

    /** Look up instPC at this level only. */
    BTBEntry *lookup(ThreadID tid, Addr instPC);

    /** Look up instPC at this level, recursing into nextLevel on a miss.
     *  On a hit in a lower level, the entry is refilled into this level
     *  (and any other inclusive level along the way) before returning.
     *  hit_latency/hit_level report the latency and index of the level
     *  that actually produced the hit, for stats/DPRINTF purposes.
     */
    BTBEntry *multiLookup(ThreadID tid, Addr instPC, Cycles &hit_latency,
                          unsigned &hit_level);

    /** Insert/update instPC's entry at this level, allocating (and
     *  possibly evicting) a slot for it if it isn't already present.
     */
    BTBEntry *insertEntry(ThreadID tid, Addr instPC, const PCStateBase &target,
                          StaticInstPtr inst);

    void doWriteback(ThreadID tid, const BTBEntry &upper_victim);

    AssociativeCache<BTBEntry> btb;
    const Cycles latency;
    const bool inclusive;
    BTBLevel *nextLevel = nullptr;
    unsigned level = 0;
};

class MultiLevelBTB : public BranchTargetBuffer
{
  public:
    MultiLevelBTB(const MultiLevelBTBParams &params);

    void memInvalidate() override;
    bool valid(ThreadID tid, Addr instPC) override;

    const BTBLookupResult
    lookup(ThreadID tid, Addr instPC,
           BranchType type = BranchType::NoBranch) override;

    void update(ThreadID tid, Addr instPC, const PCStateBase &target_pc,
                BranchType type = BranchType::NoBranch,
                StaticInstPtr inst = nullptr) override;

    const StaticInstPtr getInst(ThreadID tid, Addr instPC) override;

  private:
    const std::vector<BTBLevel *> levels;

    struct MultiLevelBTBStats : public statistics::Group
    {
        MultiLevelBTBStats(statistics::Group *parent,
                           const std::vector<BTBLevel *> &levels);

        statistics::Vector levelHits;
    } multilevelstats;
};
} // namespace gem5::branch_prediction

#endif // __CPU_PRED_MULTI_LEVEL_BTB_HH__
