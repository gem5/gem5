/*
 * Copyright (c) 2019, 2020 Inria
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

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

#include "base/sat_counter.hh"
#include "mem/cache/tags/dueling.hh"

using namespace gem5;

/**
 * Test whether dueler provides correct sampling functionality and assigns
 * teams correctly.
 */
TEST(DuelerTest, SetSample)
{
    const int num_sampled_ids = 3;
    bool team;
    Dueler dueler;

    // Initially nobody is sampled
    for (int id = 0; id < 64; id++) {
        ASSERT_FALSE(dueler.isSample(1ULL << id, team));
    }

    // Mark the first num_sampled_ids as samples and assign alternating teams
    team = false;
    for (int id = 0; id < num_sampled_ids; id++) {
        team = !team;
        dueler.setSample(1ULL << id, team);
    }

    // Make sure that only the entries marked as samples previously are samples
    bool expected_team = false;
    for (int id = 0; id < 64; id++) {
        expected_team = !expected_team;
        const bool is_sample = dueler.isSample(1ULL << id, team);
        if (id < num_sampled_ids) {
            ASSERT_TRUE(is_sample);
            ASSERT_EQ(team, expected_team);
        } else {
            ASSERT_FALSE(is_sample);
        }
    }

}

// We assume that the monitors are assigned sequential powers of
// two ids starting from 1
static uint64_t monitor_id = 0;

class DuelingMonitorTest : public testing::TestWithParam<
    std::tuple<unsigned, std::size_t, std::size_t, unsigned, double, double>>
{
  protected:
    /** A vector simulating a table-like structure. */
    std::vector<Dueler> entries;

    /** The constituency size, acquired from params. */
    std::size_t constituencySize;

    /** The size of a team, acquired from params. */
    std::size_t teamSize;

    /** The number of bits in the selector, acquired from params. */
    unsigned numBits;

    /** The low threshold to change winner, acquired from params. */
    double lowThreshold;

    /** The high threshold to change winner, acquired from params. */
    double highThreshold;

    /** The monitor instance being tested. */
    std::unique_ptr<DuelingMonitor> monitor;

    void
    SetUp() override
    {
        const unsigned num_entries = std::get<0>(GetParam());
        constituencySize = std::get<1>(GetParam());
        teamSize = std::get<2>(GetParam());
        numBits = std::get<3>(GetParam());
        lowThreshold = std::get<4>(GetParam());
        highThreshold = std::get<5>(GetParam());

        monitor.reset(new DuelingMonitor(constituencySize, teamSize, numBits,
            lowThreshold, highThreshold));

        // Initialize entries
        entries.resize(num_entries);
        for (auto& entry : entries) {
            monitor->initEntry(&entry);
        }
    }

    void
    TearDown() override
    {
        // Due to the way the DuelingMonitor works, this id is not testable
        // anymore, so move to the next
        monitor_id++;
    }
};

/**
 * Test whether entry initialization creates exactly the amount of samples
 * requested.
 */
TEST_P(DuelingMonitorTest, CountSamples)
{
    const int expected_num_samples =
        (entries.size() / constituencySize) * teamSize;

    // Check whether the number of samples match expectation.
    // Both teams must also have the same number of samples
    int count_samples_true = 0;
    int count_samples_false = 0;
    bool team;
    for (auto& entry : entries) {
        if (entry.isSample(1ULL << monitor_id, team)) {
            if (team) {
                count_samples_true++;
            } else {
                count_samples_false++;
            }
        }
    }
    ASSERT_EQ(count_samples_true, count_samples_false);
    ASSERT_EQ(count_samples_true, expected_num_samples);
}

/** Test winner selection with multiple different threshold configurations. */
TEST_P(DuelingMonitorTest, WinnerSelection)
{
    SatCounter32 expected_selector(numBits);
    expected_selector.saturate();
    expected_selector >>= 1;

    // Initialize entries, and save a pointer to a sample of each
    // team, and one no sample
    int team_true_index = -1;
    int team_false_index = -1;
    int no_sample_index = -1;
    for (int index = 0; index < entries.size(); index++) {
        bool team;
        if (entries[index].isSample(1ULL << monitor_id, team)) {
            if (team) {
                team_true_index = index;
            } else {
                team_false_index = index;
            }
        } else {
            no_sample_index = index;
        }
    }
    ASSERT_TRUE(team_true_index >= 0);
    ASSERT_TRUE(team_false_index >= 0);

    // Duel for team true only. If the monitor starts predicting false
    // the threshold to use is the higher one. Otherwise, we should
    // start expecting true to be the winner, and thus we use the
    // low threshold
    bool current_winner = monitor->getWinner();
    double threshold = current_winner ? lowThreshold : highThreshold;
    for (; expected_selector.calcSaturation() < 1.0;
        expected_selector++) {
        ASSERT_EQ(expected_selector.calcSaturation() >= threshold,
            monitor->getWinner());
        monitor->sample(&entries[team_true_index]);
    }
    current_winner = monitor->getWinner();
    ASSERT_TRUE(current_winner);

    // Duel for no sample. Should not change winner
    if (no_sample_index >= 0) {
        for (int i = 0; i < 200; i++) {
            monitor->sample(&entries[no_sample_index]);
        }
        ASSERT_EQ(current_winner, monitor->getWinner());
    }

    // Duel for team false only. Now that we know that team true
    // is winning, the low threshold must be passed in order to
    // make team false win the duel
    threshold = lowThreshold;
    for (; expected_selector.calcSaturation() > 0.0;
        expected_selector--) {
        ASSERT_EQ(expected_selector.calcSaturation() >= threshold,
            monitor->getWinner());
        monitor->sample(&entries[team_false_index]);
    }
    current_winner = monitor->getWinner();
    ASSERT_FALSE(current_winner);

    // Duel for no sample. Should not change winner
    if (no_sample_index >= 0) {
        for (int i = 0; i < 200; i++) {
            monitor->sample(&entries[no_sample_index]);
        }
        ASSERT_EQ(current_winner, monitor->getWinner());
    }
}

// Test a few possible parameter combinations. There is a limitation on
// the number of tests due to the maximum number of different ids. Because
// of that, if a new test is added and it fails for no reason, make sure
// that this limit has not been surpassed
INSTANTIATE_TEST_CASE_P(DuelingMonitorTests, DuelingMonitorTest,
    ::testing::Values(
        // Combinations of constituencies and teams
        std::make_tuple(32, 2, 1, 1, 0.5, 0.5),
        std::make_tuple(32, 4, 1, 1, 0.5, 0.5),
        std::make_tuple(32, 4, 2, 1, 0.5, 0.5),
        std::make_tuple(32, 8, 1, 1, 0.5, 0.5),
        std::make_tuple(32, 8, 2, 1, 0.5, 0.5),
        std::make_tuple(32, 8, 4, 1, 0.5, 0.5),
        std::make_tuple(32, 16, 1, 1, 0.5, 0.5),
        std::make_tuple(32, 16, 2, 1, 0.5, 0.5),
        std::make_tuple(32, 16, 4, 1, 0.5, 0.5),
        std::make_tuple(32, 16, 8, 1, 0.5, 0.5),

        // Tests for the thresholds
        std::make_tuple(16, 4, 1, 3, 0.5, 0.5),
        std::make_tuple(16, 4, 1, 3, 0.1, 0.7),
        std::make_tuple(16, 4, 1, 3, 0.4, 0.6),
        std::make_tuple(16, 4, 1, 3, 0.8, 0.9),

        // Test for larger tables
        std::make_tuple(2048, 32, 4, 4, 0.4, 0.6))
);
