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

#include "mem/cache/replacement_policies/mru_rp.hh"
#include "params/MRURP.hh"

// We need to initialize the event queue to be able to count ticks
gem5::EventQueue eventQueue("MRURPTest Queue");

/// Common fixture that initializes the replacement policy
class MRURPTestF : public ::testing::Test
{
  public:
    std::shared_ptr<gem5::replacement_policy::MRU> rp;

    MRURPTestF()
    {
        gem5::MRURPParams params;
        params.eventq_index = 0;
        rp = std::make_shared<gem5::replacement_policy::MRU>(params);

        // Assign the event queue, so that we can count ticks
        gem5::curEventQueue(&eventQueue);
    }
};

/// Test that instantiating an entry generates the replacement data of the
/// expected type
TEST_F(MRURPTestF, InstantiatedEntry)
{
    const auto repl_data = rp->instantiateEntry();

    // instantiateEntry must return a valid pointer
    ASSERT_NE(repl_data, nullptr);
}

/// Test that if there is one candidate, then it will always be the victim,
/// regardless of its replacement data
TEST_F(MRURPTestF, GetVictim1Candidate)
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
class MRURPVictimizationTestF : public MRURPTestF
{
  protected:
    // The entries being victimized
    std::vector<gem5::ReplaceableEntry> entries;

    // The entries, in candidate form
    gem5::ReplacementCandidates candidates;

  public:
    // The number of entries is arbitrary. It does not need to be high, since
    // having more entries is not expected to increase coverage
    MRURPVictimizationTestF() : MRURPTestF(), entries(4)
    {
        for (auto &entry : entries) {
            entry.replacementData = rp->instantiateEntry();
            candidates.push_back(&entry);
        }
    }
};

/// Test that when all entries are invalid a single entry will always be
/// selected, regardless of the order of the invalidations
TEST_F(MRURPVictimizationTestF, GetVictimAllInvalid)
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
TEST_F(MRURPVictimizationTestF, GetVictimOneInvalid)
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

/// Test that when there is a single MRU, set through reset(), it will be
/// the entry selected during victimization
TEST_F(MRURPVictimizationTestF, GetVictimResetOneMRU)
{
    for (auto &entry : entries) {
        // Validate all entries to start from a clean state
        for (auto &entry : entries) {
            rp->reset(entry.replacementData);
        }

        // Reset one of the entries to make it become the MRU
        eventQueue.setCurTick(gem5::curTick() + 1);
        rp->reset(entry.replacementData);

        ASSERT_EQ(rp->getVictim(candidates), &entry);
    }
}

/// Test that when there are two MRU entries, both set through reset(),
/// the first in the candidate list will always be selected during
/// victimization
TEST_F(MRURPVictimizationTestF, GetVictimResetTwoMRU)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        for (size_t j = 0; j < entries.size(); ++j) {
            // Validate all entries to start from a clean state
            for (auto &entry : entries) {
                rp->reset(entry.replacementData);
            }

            // Reset the two selected the entries to make them become the MRU
            eventQueue.setCurTick(gem5::curTick() + 1);
            rp->reset(entries[i].replacementData);
            rp->reset(entries[j].replacementData);

            // Heuristic: the first entry is the victim
            ASSERT_EQ(rp->getVictim(candidates),
                      (i < j) ? &entries[i] : &entries[j]);
        }
    }
}

/// Test that when there is a single MRU, set through touch(), it will be
/// the entry selected during victimization
TEST_F(MRURPVictimizationTestF, GetVictimTouchOneMRU)
{
    for (auto &entry : entries) {
        // Validate all entries to start from a clean state
        for (auto &entry : entries) {
            rp->reset(entry.replacementData);
        }

        // Touch one of the entries to make it become MRU
        eventQueue.setCurTick(gem5::curTick() + 1);
        rp->touch(entry.replacementData);

        ASSERT_EQ(rp->getVictim(candidates), &entry);
    }
}

/// Test that when there are two MRU entries, both set through touch(),
/// the first in the candidate list will always be selected during
/// victimization
TEST_F(MRURPVictimizationTestF, GetVictimTouchTwoMRU)
{
    for (size_t i = 0; i < entries.size(); ++i) {
        for (size_t j = 0; j < entries.size(); ++j) {
            // Validate all entries to start from a clean state
            for (auto &entry : entries) {
                rp->reset(entry.replacementData);
            }

            // Reset the two selected the entries to make them become the MRU
            eventQueue.setCurTick(gem5::curTick() + 1);
            rp->touch(entries[i].replacementData);
            rp->touch(entries[j].replacementData);

            // Heuristic: the first entry is the victim
            ASSERT_EQ(rp->getVictim(candidates),
                      (i < j) ? &entries[i] : &entries[j]);
        }
    }
}

typedef MRURPTestF MRURPFDeathTest;

TEST_F(MRURPFDeathTest, InvalidateNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->invalidate(nullptr), "");
}

TEST_F(MRURPFDeathTest, ResetNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->reset(nullptr), "");
}

TEST_F(MRURPFDeathTest, TouchNull)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    ASSERT_DEATH(rp->touch(nullptr), "");
}

TEST_F(MRURPFDeathTest, NoCandidates)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
                    "stripped out of fast builds";
#endif
    gem5::ReplacementCandidates candidates;
    ASSERT_DEATH(rp->getVictim(candidates), "");
}
