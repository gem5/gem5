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

/**
 * @file
 * Declaration of a Second-Chance replacement policy.
 * The victim is chosen using the timestamp. The oldest entry is chosen
 * to be evicted, if it hasn't been touched since its insertion. If it
 * has been touched, it is given a second chance and re-inserted at the
 * end of the queue.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SECOND_CHANCE_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SECOND_CHANCE_RP_HH__

#include "base/cache/replacement_policies/base.hh"
#include "base/cache/replacement_policies/fifo_rp.hh"

namespace gem5
{

struct SecondChanceRPParams;

namespace replacement_policy
{

class SecondChance : public FIFO
{
  protected:
    /** Second-Chance-specific implementation of replacement data. */
    struct SecondChanceReplData : public FIFOReplData
    {
        /**
         * This is different from isTouched because isTouched accounts only
         * for insertion, while this bit is reset every new re-insertion.
         * @sa SecondChance.
         */
        bool hasSecondChance;

        /**
         * Default constructor.
         */
        SecondChanceReplData() : FIFOReplData(), hasSecondChance(false) {}
    };

    /**
     * Use replacement data's second chance.
     *
     * @param replacement_data Entry that will use its second chance.
     */
    void useSecondChance(
        const std::shared_ptr<SecondChanceReplData>& replacement_data) const;

  public:
    typedef SecondChanceRPParams Params;
    SecondChance(const Params &p);
    ~SecondChance() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Invalid entries do not have a second chance, and their last touch tick
     * is set as the oldest possible.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its re-insertion tick and second chance bit.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted or re-inserted
     * in the queue.
     * Sets its insertion tick and second chance bit.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using insertion timestamps and second chance
     * bit.
     *
     * @param cands Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SECOND_CHANCE_RP_HH__
