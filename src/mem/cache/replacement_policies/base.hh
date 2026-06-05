/**
 * Copyright (c) 2018-2020 Inria
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

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_BASE_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_BASE_HH__

#include <memory>

#include "base/compiler.hh"
#include "base/statistics.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/packet.hh"
#include "params/BaseReplacementPolicy.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * Replacement candidates as chosen by the indexing policy.
 */
typedef std::vector<ReplaceableEntry*> ReplacementCandidates;

namespace replacement_policy
{

/**
 * A common base class of cache replacement policy objects.
 *
 * Concrete policies do not override the public interface directly. Instead,
 * they implement the protected @c *Impl methods, while the public methods
 * (invalidate, touch, reset and getVictim) are non-virtual wrappers provided
 * by this base class. These wrappers gather profiling statistics common to
 * every policy and then forward to the policy-specific implementation. This
 * Non-Virtual Interface keeps statistics collection in a single place, so no
 * policy can accidentally bypass it.
 */
class Base : public SimObject
{
  protected:
    /** Profiling information shared by all replacement policies. */
    struct ReplacementStats : public statistics::Group
    {
        ReplacementStats(statistics::Group *parent)
            : statistics::Group(parent),
              ADD_STAT(victimizations, statistics::units::Count::get(),
                       "Number of entries selected for eviction by getVictim"),
              ADD_STAT(hits, statistics::units::Count::get(),
                       "Number of accesses to entries already present"),
              ADD_STAT(insertions, statistics::units::Count::get(),
                       "Number of entries inserted/validated"),
              ADD_STAT(invalidations, statistics::units::Count::get(),
                       "Number of entries invalidated"),
              ADD_STAT(
                  avgHitsPerInsertion, statistics::units::Ratio::get(),
                  "Average number of hits seen by an entry before replacement")
        { avgHitsPerInsertion = hits / insertions; }

        /** Number of entries selected for eviction by getVictim. */
        statistics::Scalar victimizations;
        /** Number of accesses to entries already present (cache hits). */
        statistics::Scalar hits;
        /** Number of entries inserted/validated. */
        statistics::Scalar insertions;
        /** Number of entries invalidated. */
        statistics::Scalar invalidations;
        /** Average number of hits seen by an entry before it is replaced. */
        statistics::Formula avgHitsPerInsertion;
    };

    /**
     * Profiling counters. Marked mutable because the const interface methods
     * (e.g. touch and getVictim) must be able to update them.
     */
    mutable ReplacementStats stats;

  public:
    typedef BaseReplacementPolicyParams Params;
    Base(const Params &p) : SimObject(p), stats(this) {}
    virtual ~Base() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void
    invalidate(const std::shared_ptr<ReplacementData> &replacement_data)
    {
        stats.invalidations++;
        invalidateImpl(replacement_data);
    }

    /**
     * Update replacement data.
     *
     * @param replacement_data Replacement data to be touched.
     * @param pkt Packet that generated this access.
     */
    void
    touch(const std::shared_ptr<ReplacementData> &replacement_data,
          const PacketPtr pkt)
    {
        stats.hits++;
        touchImpl(replacement_data, pkt);
    }
    void
    touch(const std::shared_ptr<ReplacementData> &replacement_data) const
    {
        stats.hits++;
        touchImpl(replacement_data);
    }

    /**
     * Reset replacement data. Used when it's holder is inserted/validated.
     *
     * @param replacement_data Replacement data to be reset.
     * @param pkt Packet that generated this access.
     */
    void
    reset(const std::shared_ptr<ReplacementData> &replacement_data,
          const PacketPtr pkt)
    {
        stats.insertions++;
        resetImpl(replacement_data, pkt);
    }
    void
    reset(const std::shared_ptr<ReplacementData> &replacement_data) const
    {
        stats.insertions++;
        resetImpl(replacement_data);
    }

    /**
     * Find replacement victim among candidates.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry *
    getVictim(const ReplacementCandidates &candidates) const
    {
        stats.victimizations++;
        return getVictimImpl(candidates);
    }

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    virtual std::shared_ptr<ReplacementData> instantiateEntry() = 0;

  protected:
    /**
     * Policy-specific implementation of the public interface. See the
     * corresponding public methods for the expected behavior.
     * @{
     */
    virtual void invalidateImpl(
        const std::shared_ptr<ReplacementData> &replacement_data) = 0;

    virtual void
    touchImpl(const std::shared_ptr<ReplacementData> &replacement_data,
              const PacketPtr pkt)
    { touchImpl(replacement_data); }
    virtual void touchImpl(
        const std::shared_ptr<ReplacementData> &replacement_data) const = 0;

    virtual void
    resetImpl(const std::shared_ptr<ReplacementData> &replacement_data,
              const PacketPtr pkt)
    { resetImpl(replacement_data); }
    virtual void resetImpl(
        const std::shared_ptr<ReplacementData> &replacement_data) const = 0;

    virtual ReplaceableEntry *
    getVictimImpl(const ReplacementCandidates &candidates) const = 0;
    /** @} */
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_BASE_HH__
