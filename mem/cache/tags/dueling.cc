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

#include "mem/cache/tags/dueling.hh"

#include "base/bitfield.hh"
#include "base/logging.hh"

namespace gem5
{

unsigned DuelingMonitor::numInstances = 0;

Dueler::Dueler()
  : _isSample(false), _team(0)
{
}

void
Dueler::setSample(uint64_t id, bool team)
{
    panic_if(popCount(id) != 1, "The id must have a single bit set.");
    panic_if(_isSample & id,
        "This dueler is already a sample for id %llu", id);
    _isSample |= id;
    if (team) {
        _team |= id;
    }
}

bool
Dueler::isSample(uint64_t id, bool& team) const
{
    team = _team & id;
    return _isSample & id;
}

DuelingMonitor::DuelingMonitor(std::size_t constituency_size,
    std::size_t team_size, unsigned num_bits, double low_threshold,
    double high_threshold)
  : id(1 << numInstances), constituencySize(constituency_size),
    teamSize(team_size), lowThreshold(low_threshold),
    highThreshold(high_threshold), selector(num_bits), regionCounter(0),
    winner(true)
{
    fatal_if(constituencySize < (NUM_DUELERS * teamSize),
        "There must be at least team size entries per team in a constituency");
    fatal_if(numInstances > 63, "Too many Dueling instances");
    fatal_if((lowThreshold <= 0.0) || (highThreshold >= 1.0),
        "The low threshold must be within the range ]0.0, 1.0[");
    fatal_if((highThreshold <= 0.0) || (highThreshold >= 1.0),
        "The high threshold must be within the range ]0.0, 1.0[");
    fatal_if(lowThreshold > highThreshold,
        "The low threshold must be below the high threshold");
    numInstances++;

    // Start selector around its middle value
    selector.saturate();
    selector >>= 1;
    if (selector.calcSaturation() < lowThreshold) {
        winner = false;
    }
}

void
DuelingMonitor::sample(const Dueler* dueler)
{
    bool team;
    if (dueler->isSample(id, team)) {
        if (team) {
            selector++;

            if (selector.calcSaturation() >= highThreshold) {
                winner = true;
            }
        } else {
            selector--;

            if (selector.calcSaturation() < lowThreshold) {
                winner = false;
            }
        }
    }
}

bool
DuelingMonitor::isSample(const Dueler* dueler, bool& team) const
{
    return dueler->isSample(id, team);
}

bool
DuelingMonitor::getWinner() const
{
    return winner;
}

void
DuelingMonitor::initEntry(Dueler* dueler)
{
    // The first entries of the constituency belong to one team, and the
    // last entries to the other
    assert(dueler);
    if (regionCounter < teamSize) {
        dueler->setSample(id, false);
    } else if (regionCounter >= constituencySize - teamSize) {
        dueler->setSample(id, true);
    }

    // Check if we changed constituencies
    if (++regionCounter >= constituencySize) {
        regionCounter = 0;
    }
}

} // namespace gem5
