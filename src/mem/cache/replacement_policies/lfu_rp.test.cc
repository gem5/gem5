/**
 * Copyright (c) 2025 Daniel R. Carvalho
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

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <cassert>

#include "mem/cache/replacement_policies/lfu_rp.hh"
#include "params/LFURP.hh"

/// Common fixture that initializes the replacement policy
class LFURPTestF : public ::testing::Test
{
  public:
    std::shared_ptr<gem5::replacement_policy::LFU> rp;

    LFURPTestF()
    {
        gem5::LFURPParams params;
        params.eventq_index = 0;
        rp = std::make_shared<gem5::replacement_policy::LFU>(params);
    }
};

/// Test that instantiating an entry generates a replacement data
TEST_F(LFURPTestF, InstantiatedEntry)
{
    const auto repl_data = rp->instantiateEntry();

    // instantiateEntry must return a valid pointer
    ASSERT_NE(repl_data, nullptr);
}

/// Test that if there is one candidate, then it will always be the victim,
/// regardless of its replacement data
TEST_F(LFURPTestF, GetVictim1Candidate)
{
    gem5::ReplaceableEntry entry;
    entry.replacementData = rp->instantiateEntry();
    gem5::ReplacementCandidates candidates;
    candidates.push_back(&entry);
    ASSERT_EQ(rp->getVictim(candidates), &entry);

    rp->invalidate(entry.replacementData);
    ASSERT_EQ(rp->getVictim(candidates), &entry);

    rp->reset(entry.replacementData);
    ASSERT_EQ(rp->getVictim(candidates), &entry);

    rp->touch(entry.replacementData);
    ASSERT_EQ(rp->getVictim(candidates), &entry);
}

/// Fixture that tests victimization
class LFURPVictimizationTestF : public LFURPTestF
{
  protected:
    // The entries being victimized
    std::vector<gem5::ReplaceableEntry> entries;

    // The entries, in candidate form
    gem5::ReplacementCandidates candidates;

  public:
    // The number of entries is arbitrary. It does not need to be high, since
    // having more entries is not expected to increase coverage
    LFURPVictimizationTestF() : LFURPTestF(), entries(4)
    {
        for (auto &entry : entries) {
            entry.replacementData = rp->instantiateEntry();
            candidates.push_back(&entry);
        }
    }
};

/// Test that when all entries are invalid a single entry will always be
/// selected, regardless of the order of the invalidations
TEST_F(LFURPVictimizationTestF, GetVictimAllInvalid)
{
    auto expected_victim = &entries[0];

    // At this point all candidates are invalid
    ASSERT_EQ(rp->getVictim(candidates), expected_victim);

    // Since all candidates are already invalid, nothing changes if we
    // invalidate them again
    for (auto &entry : entries) {
        rp->invalidate(entry.replacementData);
    }
    ASSERT_EQ(rp->getVictim(candidates), expected_victim);

    // Even if we invalidate the entry being selected for victimization last
    rp->invalidate(expected_victim->replacementData);
    ASSERT_EQ(rp->getVictim(candidates), expected_victim);
}

/// Test that when there is at least a single invalid entry, it will be
/// selected during the victimization
TEST_F(LFURPVictimizationTestF, GetVictimOneInvalid)
{
    for (auto &entry : entries) {
        // Validate all entries to start from a clean state
        for (auto &entry : entries) {
            rp->reset(entry.replacementData);
        }

        // Set one of the entries as invalid
        rp->invalidate(entry.replacementData);

        ASSERT_EQ(rp->getVictim(candidates), &entry);
    }
}

/// Test that when all entries have been reset the same entry will always
/// be the one victimized
TEST_F(LFURPVictimizationTestF, GetVictimAllReset)
{
    for (auto &entry : entries) {
        rp->reset(entry.replacementData);
    }

    ASSERT_EQ(rp->getVictim(candidates), &entries[0]);
}

/// Test that when there is a single LFU, set through touch(), it will be
/// the entry selected during victimization
TEST_F(LFURPVictimizationTestF, GetVictimTouchOneLFU)
{
    // Test a couple of different touching frequencies
    for (unsigned freq = 1; freq < 5; ++freq) {
        for (size_t i = 0; i < entries.size(); ++i) {
            auto &entry = entries[i];

            // Reset all entries and touch all the entries other than the entry
            // of interest at least as many times as the frequency specifies
            for (size_t j = 0; j < entries.size(); ++j) {
                rp->reset(entries[j].replacementData);

                if (i != j) {
                    for (size_t f = 0; f < freq; ++f) {
                        rp->touch(entries[j].replacementData);
                    }
                }
            }

            // Touch the entry of interest less times than we have touched
            // the other entries
            for (size_t f = 0; f < freq - 1; ++f) {
                rp->touch(entry.replacementData);
            }

            ASSERT_EQ(rp->getVictim(candidates), &entry);
        }
    }
}

/// Test that when there are two LFU entries, both set through touch(),
/// the first in the candidate list will always be selected during
/// victimization
TEST_F(LFURPVictimizationTestF, GetVictimTouchTwoLFU)
{
    // Test a couple of different touching frequencies
    for (unsigned freq = 1; freq < 5; ++freq) {
        SCOPED_TRACE(freq);
        for (size_t i = 0; i < entries.size(); ++i) {
            SCOPED_TRACE(i);
            for (size_t j = 0; j < entries.size(); ++j) {
                SCOPED_TRACE(j);
                // Reset all entries and touch all the entries other than
                // the entry of interest at least as many times as the
                // frequency specifies
                for (size_t k = 0; k < entries.size(); ++k) {
                    rp->reset(entries[k].replacementData);

                    if ((k != i) && (k != j)) {
                        for (size_t f = 0; f < freq; ++f) {
                            rp->touch(entries[k].replacementData);
                        }
                    }
                }

                // Touch the entries of interest less times than we have
                // touched the other entries
                for (size_t f = 0; f < freq - 1; ++f) {
                    rp->touch(entries[i].replacementData);
                    if (i != j) {
                        rp->touch(entries[j].replacementData);
                    }
                }

                // Heuristic: the first entry is the victim
                ASSERT_EQ(rp->getVictim(candidates),
                          (i < j) ? &entries[i] : &entries[j]);
            }
        }
    }
}

typedef LFURPTestF LFURPFDeathTest;

TEST_F(LFURPFDeathTest, InvalidateNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->invalidate(nullptr), "");
}

TEST_F(LFURPFDeathTest, ResetNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->reset(nullptr), "");
}

TEST_F(LFURPFDeathTest, TouchNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->touch(nullptr), "");
}

TEST_F(LFURPFDeathTest, NoCandidates)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    gem5::ReplacementCandidates candidates;
    ASSERT_DEATH(rp->getVictim(candidates), "");
}
