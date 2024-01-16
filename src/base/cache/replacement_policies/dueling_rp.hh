/**
 * Copyright (c) 2019, 2020 Inria
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

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_DUELING_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_DUELING_RP_HH__

#include <memory>

#include "base/cache/replacement_policies/base.hh"
#include "base/compiler.hh"
#include "base/statistics.hh"
#include "mem/cache/tags/dueling.hh"

namespace gem5
{

struct DuelingRPParams;

namespace replacement_policy
{

/**
 * This replacement policy duels two replacement policies to find out which
 * one provides the best results. A policy is said to have the best results
 * when it has a lower number of misses.
 */
class Dueling : public Base
{
  protected:
    /**
     * Dueler-specific implementation of replacement data. Contains all
     * sub-replacement policies' replacement data.
     */
    struct DuelerReplData : ReplacementData, Dueler
    {
        std::shared_ptr<ReplacementData> replDataA;
        std::shared_ptr<ReplacementData> replDataB;

        /** Default constructor. Initialize sub-replacement data. */
        DuelerReplData(const std::shared_ptr<ReplacementData>& repl_data_a,
            const std::shared_ptr<ReplacementData>& repl_data_b)
          : ReplacementData(), Dueler(), replDataA(repl_data_a),
            replDataB(repl_data_b)
        {
        }
    };

    /** Sub-replacement policy used in this multiple container. */
    Base* const replPolicyA;
    /** Sub-replacement policy used in this multiple container. */
    Base* const replPolicyB;

    /**
     * A dueling monitor that decides which is the best sub-policy based on
     * their number of misses.
     */
    mutable DuelingMonitor duelingMonitor;

    mutable struct DuelingStats : public statistics::Group
    {
        DuelingStats(statistics::Group* parent);

        /** Number of times A was selected on victimization. */
        statistics::Scalar selectedA;

        /** Number of times B was selected on victimization. */
        statistics::Scalar selectedB;
    } duelingStats;

  public:
    PARAMS(DuelingRP);
    Dueling(const Params &p);
    ~Dueling() = default;

    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;
    void touch(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_DUELING_RP_HH__
