/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 *
 * Authors: Andrew Bardsley
 *          Matthias Jung
 *          Abdul Mutaal Ahmad
 */

/**
 * @file
 *
 *  C++-only configuration stats handling example
 *
 *  Register with: Stats::registerHandlers(statsReset, statsDump)
 */

#include "base/output.hh"
#include "base/statistics.hh"
#include "base/stats/text.hh"
#include "stats.hh"

namespace CxxConfig
{

void statsPrepare()
{
    std::list<Stats::Info *> stats = Stats::statsList();

    /* gather_stats -> prepare */
    for (auto i = stats.begin(); i != stats.end(); ++i){
        Stats::Info *stat = *i;
        Stats::VectorInfo *vector = dynamic_cast<Stats::VectorInfo *>(stat);
        if (vector){
            (dynamic_cast<Stats::VectorInfo *>(*i))->prepare();
        }
        else {
            (*i)->prepare();
        }

    }
}

void statsDump()
{
    bool desc = true;
    Stats::Output *output = Stats::initText(filename, desc);

    Stats::processDumpQueue();

    std::list<Stats::Info *> stats = Stats::statsList();

    statsEnable();
    statsPrepare();

    output->begin();
    /* gather_stats -> convert_value */
    for (auto i = stats.begin(); i != stats.end(); ++i) {
        Stats::Info *stat = *i;

        const Stats::ScalarInfo *scalar = dynamic_cast<Stats::ScalarInfo
            *>(stat);
        Stats::VectorInfo *vector = dynamic_cast<Stats::VectorInfo *>(stat);
        const Stats::Vector2dInfo *vector2d = dynamic_cast<Stats::Vector2dInfo
            *>(vector);
        const Stats::DistInfo *dist = dynamic_cast<Stats::DistInfo *>(stat);
        const Stats::VectorDistInfo *vectordist =
            dynamic_cast<Stats::VectorDistInfo *>(stat);
        const Stats::SparseHistInfo *sparse =
            dynamic_cast<Stats::SparseHistInfo *>(stat);
        const Stats::InfoProxy <Stats::Vector2d,Stats::Vector2dInfo> *info =
            dynamic_cast<Stats::InfoProxy
            <Stats::Vector2d,Stats::Vector2dInfo>*>(stat);

        if (vector) {
            const Stats::FormulaInfo *formula = dynamic_cast<Stats::FormulaInfo
                *>(vector);
            if (formula){
                output->visit(*formula);
            } else {
                const Stats::VectorInfo *vector1 = vector;
                output->visit(*vector1);
            }
        } else if (vector2d) {
            output->visit(*vector2d);
        } else if (info){
            output->visit(*info);
        } else if (vectordist){
            output->visit(*vectordist);
        } else if (dist) {
            output->visit(*dist);
        } else if (sparse) {
            output->visit(*sparse);
        } else if (scalar) {
            output->visit(*scalar);
        } else {
            warn("Stat not dumped: %s\n", stat->name);
        }
    }
    output->end();
}

void statsReset()
{
    std::cerr << "Stats reset\n";

    Stats::processResetQueue();
}

void statsEnable()
{
    std::list<Stats::Info *> stats = Stats::statsList();

    for (auto i = stats.begin(); i != stats.end(); ++i){
        Stats::Info *stat = *i;
        Stats::VectorInfo *vector = dynamic_cast<Stats::VectorInfo *>(stat);
        if (vector){
            (dynamic_cast<Stats::VectorInfo *>(*i))->enable();
        }
        else {
            (*i)->enable();
        }

    }
}

}
