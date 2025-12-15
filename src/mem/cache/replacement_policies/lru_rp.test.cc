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

#include "mem/cache/replacement_policies/lru_rp.hh"
#include "params/LRURP.hh"

// We need to initialize the event queue to be able to count ticks
gem5::EventQueue eventQueue("MRURPTest Queue");

/// Common fixture that initializes the replacement policy
class LRURPTestF : public ::testing::Test
{
  public:
    std::shared_ptr<gem5::replacement_policy::LRU> rp;

    LRURPTestF()
    {
        gem5::LRURPParams params;
        params.eventq_index = 0;
        rp = std::make_shared<gem5::replacement_policy::LRU>(params);

        // Assign the event queue, so that we can count ticks
        gem5::curEventQueue(&eventQueue);
    }
};

/// Test that instantiating an entry generates a replacement data
TEST_F(LRURPTestF, InstantiatedEntry)
{
    const auto repl_data = rp->instantiateEntry();

    // instantiateEntry must return a valid pointer
    ASSERT_NE(repl_data, nullptr);
}

/// Test that if there is one candidate, then it will always be the victim,
/// regardless of its replacement data
TEST_F(LRURPTestF, GetVictim1Candidate)
{
    gem5::ReplaceableEntry entry;
    entry.replacementData = rp->instantiateEntry();
    gem5::ReplacementCandidates candidates;
    candidates.push_back(&entry);
    ASSERT_EQ(rp->getVictim(candidates), &entry);
    eventQueue.setCurTick(gem5::curTick() + 1);

    rp->invalidate(entry.replacementData);
    ASSERT_EQ(rp->getVictim(candidates), &entry);
    eventQueue.setCurTick(gem5::curTick() + 1);

    rp->reset(entry.replacementData);
    ASSERT_EQ(rp->getVictim(candidates), &entry);
    eventQueue.setCurTick(gem5::curTick() + 1);

    rp->touch(entry.replacementData);
    ASSERT_EQ(rp->getVictim(candidates), &entry);
    eventQueue.setCurTick(gem5::curTick() + 1);
}

/// Fixture that tests victimization
class LRURPVictimizationTestF : public LRURPTestF
{
  protected:
    // The entries being victimized
    std::vector<gem5::ReplaceableEntry> entries;

    // The entries, in candidate form
    gem5::ReplacementCandidates candidates;

  public:
    // The number of entries is arbitrary. It does not need to be high, since
    // having more entries is not expected to increase coverage
    LRURPVictimizationTestF() : LRURPTestF(), entries(4)
    {
        for (auto &entry : entries) {
            entry.replacementData = rp->instantiateEntry();
            candidates.push_back(&entry);
        }
    }
};

/// Test that when all entries are invalid a single entry will always be
/// selected, regardless of the order of the invalidations
TEST_F(LRURPVictimizationTestF, GetVictimAllInvalid)
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
    eventQueue.setCurTick(gem5::curTick() + 1);
    rp->invalidate(expected_victim->replacementData);
    ASSERT_EQ(rp->getVictim(candidates), expected_victim);
}

/// Test that when there is at least a single invalid entry, it will be
/// selected during the victimization
TEST_F(LRURPVictimizationTestF, GetVictimOneInvalid)
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

/// Test that when all entries have been reset the oldest entry will always
/// be the one victimized. It also verifies that touching the other entries
/// will not change the outcome.
TEST_F(LRURPVictimizationTestF, GetVictimAllReset)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);

        // Make this entry the oldest by resetting it before all others
        rp->reset(entries[i].replacementData);
        eventQueue.setCurTick(gem5::curTick() + 1);

        // Now reset all others
        for (size_t j = 0; j < entries.size(); ++j) {
            if (i != j) {
                rp->reset(entries[j].replacementData);
            }
        }

        // This entry will be the victim regardless of how many times the
        // victimization function is called
        for (unsigned rep = 0; rep <= entries.size(); ++rep) {
            SCOPED_TRACE(rep);
            ASSERT_EQ(rp->getVictim(candidates), &entries[i]);
        }

        // This is still true if we touch all others again after a tick
        eventQueue.setCurTick(gem5::curTick() + 1);
        for (size_t j = 0; j < entries.size(); ++j) {
            if (i != j) {
                rp->touch(entries[j].replacementData);
            }
        }

        // This entry will be the victim regardless of how many times the
        // victimization function is called
        for (unsigned rep = 0; rep <= entries.size(); ++rep) {
            SCOPED_TRACE(rep);
            ASSERT_EQ(rp->getVictim(candidates), &entries[i]);
        }
    }
}

/// Test that resetting an entry makes it no longer the LRU
TEST_F(LRURPVictimizationTestF, GetVictimResetLRU)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);

        // Make this entry the oldest by resetting it before all others
        rp->reset(entries[i].replacementData);
        eventQueue.setCurTick(gem5::curTick() + 1);

        // Now reset all others
        for (size_t j = 0; j < entries.size(); ++j) {
            if (i != j) {
                rp->reset(entries[j].replacementData);
            }
        }

        // Reset the victim in a later tick to make it no longer the victim
        eventQueue.setCurTick(gem5::curTick() + 1);
        rp->reset(entries[i].replacementData);

        ASSERT_NE(rp->getVictim(candidates), &entries[i]);
    }
}

/// Test that touching an entry makes it no longer the LRU
TEST_F(LRURPVictimizationTestF, GetVictimTouchLRU)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);

        // Make this entry the oldest by resetting it before all others
        rp->reset(entries[i].replacementData);
        eventQueue.setCurTick(gem5::curTick() + 1);

        // Now reset all others
        for (size_t j = 0; j < entries.size(); ++j) {
            if (i != j) {
                rp->reset(entries[j].replacementData);
            }
        }

        // Touch the victim in a later tick to make it no longer the victim
        eventQueue.setCurTick(gem5::curTick() + 1);
        rp->touch(entries[i].replacementData);

        ASSERT_NE(rp->getVictim(candidates), &entries[i]);
    }
}

/// Test that when there are two LRU entries the first in the candidate
/// list will always be selected during victimization
TEST_F(LRURPVictimizationTestF, GetVictimTouchTwoLRU)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        SCOPED_TRACE(i);
        for (size_t j = 0; j < entries.size(); ++j) {
            SCOPED_TRACE(j);

            // Make these entries tie for oldest by resetting them before
            // all others
            rp->reset(entries[i].replacementData);
            rp->reset(entries[j].replacementData);
            eventQueue.setCurTick(gem5::curTick() + 1);

            // Reset all other entries
            for (size_t k = 0; k < entries.size(); ++k) {
                if ((k != i) && (k != j)) {
                    rp->reset(entries[k].replacementData);
                }
            }

            // Heuristic: the first entry is the victim
            ASSERT_EQ(rp->getVictim(candidates),
                      (i < j) ? &entries[i] : &entries[j]);
        }
    }
}

typedef LRURPTestF LRURPFDeathTest;

TEST_F(LRURPFDeathTest, InvalidateNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->invalidate(nullptr), "");
}

TEST_F(LRURPFDeathTest, ResetNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->reset(nullptr), "");
}

TEST_F(LRURPFDeathTest, TouchNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->touch(nullptr), "");
}

TEST_F(LRURPFDeathTest, NoCandidates)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    gem5::ReplacementCandidates candidates;
    ASSERT_DEATH(rp->getVictim(candidates), "");
}
