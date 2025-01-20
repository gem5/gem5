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

#include "mem/cache/replacement_policies/second_chance_rp.hh"
#include "params/SecondChanceRP.hh"

/// Common fixture that initializes the replacement policy
class SecondChanceRPTestF : public ::testing::Test
{
  public:
    std::shared_ptr<gem5::replacement_policy::SecondChance> rp;

    SecondChanceRPTestF()
    {
        gem5::SecondChanceRPParams params;
        params.eventq_index = 0;
        rp = std::make_shared<gem5::replacement_policy::SecondChance>(params);
    }
};

/// Test that instantiating an entry generates a replacement data
TEST_F(SecondChanceRPTestF, InstantiatedEntry)
{
    const auto repl_data = rp->instantiateEntry();

    // instantiateEntry must return a valid pointer
    ASSERT_NE(repl_data, nullptr);
}

/// Test that if there is one candidate, then it will always be the victim,
/// regardless of its replacement data
TEST_F(SecondChanceRPTestF, GetVictim1Candidate)
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
class SecondChanceRPVictimizationTestF : public SecondChanceRPTestF
{
  protected:
    // The entries being victimized
    std::vector<gem5::ReplaceableEntry> entries;

    // The entries, in candidate form
    gem5::ReplacementCandidates candidates;

  public:
    // The number of entries is arbitrary. It does not need to be high, since
    // having more entries is not expected to increase coverage
    SecondChanceRPVictimizationTestF() : SecondChanceRPTestF(), entries(4)
    {
        for (auto &entry : entries) {
            entry.replacementData = rp->instantiateEntry();
            candidates.push_back(&entry);
        }
    }
};

/// Test that when all entries are invalid a single entry will always be
/// selected, regardless of the order of the invalidations
TEST_F(SecondChanceRPVictimizationTestF, GetVictimAllInvalid)
{
    auto expected_victim = &entries[0];

    // At this point all candidates are considered to be first, since
    // no entries have ever been reset
    ASSERT_EQ(rp->getVictim(candidates), expected_victim);

    // Since all candidates are already invalid, nothing changes if we
    // invalidate all of them again
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
TEST_F(SecondChanceRPVictimizationTestF, GetVictimResetOneInvalid)
{
    for (auto &entry : entries) {
        // Validate all entries to start from a clean state
        for (auto &entry : entries) {
            rp->reset(entry.replacementData);
        }

        // Set one of the entries as invalid and verify that it was the
        // one selected during the victimization
        rp->invalidate(entry.replacementData);
        ASSERT_EQ(rp->getVictim(candidates), &entry);
    }
}

/// Test that when there is at least a single invalid entry, and there
/// is at least one entry with a 2nd chance, the invalid entry will be
/// selected during the victimization
TEST_F(SecondChanceRPVictimizationTestF, GetVictimOneInvalidOneHas2ndChance)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);
        auto &inv_entry = entries[i];

        for (size_t j = 0; j < entries.size(); ++j) {
            SCOPED_TRACE(j);
            auto &touched_entry = entries[j];

            // This test expects 2 different entries
            if (i == j) {
                continue;
            }

            // Validate all entries to start from a clean state
            for (auto &entry : entries) {
                rp->reset(entry.replacementData);
            }

            // Give the non-invalid entry a 2nd chance
            rp->touch(touched_entry.replacementData);

            // Set one of the entries as invalid
            rp->invalidate(inv_entry.replacementData);

            ASSERT_EQ(rp->getVictim(candidates), &inv_entry);
        }
    }
}

/// Test that the first entry to be reset will be selected during victimization
TEST_F(SecondChanceRPVictimizationTestF, GetVictim)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);
        auto &entry = entries[i];

        // Reset one of the entries to make it become the single first entry
        rp->reset(entry.replacementData);

        // Now change ticks and validate all other entries to make them not
        // tie for first entry
        for (size_t k = 0; k < entries.size(); ++k) {
            if (k != i) {
                rp->reset(entries[k].replacementData);
            }
        }

        ASSERT_EQ(rp->getVictim(candidates), &entry);
    }
}

/// Test that the when an entry is touched it is given a 2nd chance, so
/// it is no longer the victim
TEST_F(SecondChanceRPVictimizationTestF, GetVictimAfterTouch)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);
        for (size_t j = 0; j < entries.size(); ++j) {
            SCOPED_TRACE(j);

            // This test expects 2 different entries
            if (i == j) {
                continue;
            }

            // Reset one of the entries to make it become first entry
            rp->reset(entries[i].replacementData);

            // Reset the second tracked entry to make it become the 2nd entry
            rp->reset(entries[j].replacementData);

            // Now change ticks and validate all other entries to make them not
            // tie for first entry
            for (size_t k = 0; k < entries.size(); ++k) {
                if ((k != i) && (k != j)) {
                    rp->reset(entries[k].replacementData);
                }
            }

            // Since no entries have been touched, they do not have a
            // second chance
            EXPECT_EQ(rp->getVictim(candidates), &entries[i]);

            // If we touch the victim entry it will be given a 2nd chance,
            // so it will no longer be the victim
            rp->touch(entries[i].replacementData);
            ASSERT_NE(rp->getVictim(candidates), &entries[i]);
            ASSERT_EQ(rp->getVictim(candidates), &entries[j]);
        }
    }
}


/// Test that the when an entry is touched it is given a 2nd chance and
/// it becomes the last entry to be victimized
TEST_F(SecondChanceRPVictimizationTestF, GetVictimTouchWraparound)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);

        auto &victim = entries[i];

        // Reset one of the entries to make it become first entry
        rp->reset(victim.replacementData);

        // Now change ticks and validate all other entries to make them not
        // tie for first entry
        for (size_t k = 0; k < entries.size(); ++k) {
            if (k != i) {
                rp->reset(entries[k].replacementData);
            }
        }

        // Since no entries have been touched, they do not have a
        // second chance
        EXPECT_EQ(rp->getVictim(candidates), &victim);

        // If we touch the victim entry it will be given a 2nd chance,
        // so it will no longer be the victim
        rp->touch(victim.replacementData);

        // Now keep touching the other entries until the original entry
        // goes back to being the first in the FIFO. Until then the
        // original victim is not the victim
        for (size_t k = 0; k < entries.size(); ++k) {
            if (k != i) {
                ASSERT_NE(rp->getVictim(candidates), &victim);
                rp->touch(entries[k].replacementData);
            }
        }

        // Now that all other entries have been touched, the original
        // victim is the victim again
        ASSERT_EQ(rp->getVictim(candidates), &victim);
    }
}

/// Test that when all the entries have a 2nd chance, the first entry to be
/// inserted is the one victimized, and their 2nd chances are spent in the
/// process. In theory, this test behaves equal to GetVictimTouchWraparound
TEST_F(SecondChanceRPVictimizationTestF, GetVictimAll2ndChance)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);
        for (size_t j = 0; j < entries.size(); ++j) {
            SCOPED_TRACE(j);

            // This test expects 2 different entries
            if (i == j) {
                continue;
            }

            // Reset one of the entries to make it become first entry
            rp->reset(entries[i].replacementData);

            // Reset the second tracked entry to make it become the 2nd entry
            rp->reset(entries[j].replacementData);

            // Now change ticks and validate all other entries to make them not
            // tie for first entry
            for (size_t k = 0; k < entries.size(); ++k) {
                if ((k != i) && (k != j)) {
                    rp->reset(entries[k].replacementData);
                }
            }

            // Give all entries a 2nd chance. Note that touching the entries
            // does not immediately update their order in the queue, so
            // entries[i] is still the oldest entry, and entries[j] is the
            // 2nd oldest
            for (size_t k = 0; k < entries.size(); ++k) {
                rp->touch(entries[k].replacementData);
            }

            // Since all entries have been touched, they all have a 2nd chance.
            // Thus, in order to find a victim, all the 2nd chances are spent.
            ASSERT_EQ(rp->getVictim(candidates), &entries[i]);

            // By now all entries have spent their 2nd chances, so if we
            // give a 2nd chance to the former oldest entry, it will no longer
            // be the victim. The former 2nd oldest is now the victim, as it
            // does not have a 2nd chance
            rp->touch(entries[i].replacementData);
            ASSERT_NE(rp->getVictim(candidates), &entries[i]);
            ASSERT_EQ(rp->getVictim(candidates), &entries[j]);
        }
    }
}

typedef SecondChanceRPTestF SecondChanceRPFDeathTest;

TEST_F(SecondChanceRPFDeathTest, InvalidateNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->invalidate(nullptr), "");
}

TEST_F(SecondChanceRPFDeathTest, ResetNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->reset(nullptr), "");
}

TEST_F(SecondChanceRPFDeathTest, TouchNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->touch(nullptr), "");
}

TEST_F(SecondChanceRPFDeathTest, NoCandidates)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    gem5::ReplacementCandidates candidates;
    ASSERT_DEATH(rp->getVictim(candidates), "");
}
