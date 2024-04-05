/*
 * Copyright (c) 2024 The Regents of the University of California
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

#ifndef __STAT_TESTER_HH__
#define __STAT_TESTER_HH__

#include "base/statistics.hh"
#include "params/ScalarStatTester.hh"
#include "params/StatTester.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/**
 * This classes are used to test the stats system from setting through to
 * output. The value of the stats are set via the parameters and when
 * included in a simulation will create these stats and update them at Tick
 * zero.
 *
 * Example usage
 * -------------
 *
 *     import m5
 *     from m5.objects import (
 *         Root,
 *         ScalarStatTester,
 *     )
 *
 *     stat_tester = ScalarStatTester()
 *
 *     stat_tester.value = 42
 *     stat_tester.name = "example"
 *     root = Root(full_system=False)
 *    root.stat_tester = stat_tester
 *
 *     m5.instantiate()
 *     m5.simulate()
 *
 * When the above is run, the m5out/stats.txt file will contain the
 * following:
 *
 *     ---------- Begin Simulation Statistics ----------
 *     simSeconds                               18446744.073710
 *     simTicks                                 18446744073709551616
 *     finalTick                                18446744073709551616
 *     simFreq                                  1000000000000
 *     hostSeconds                                      0.00
 *     hostTickRate                             101355736668733798612992
 *     hostMemory                                  403847696
 *     stat_tester.example                                42
 *     ---------- End Simulation Statistics   ----------
 */
class StatTester : public SimObject
{
  private:
    EventFunctionWrapper event;

  public:
    StatTester(const StatTesterParams &p) :
        SimObject(p),
        event([this]{ setStats(); }, name() + ".event")
    {}
    void startup() override { schedule(event, curTick()); }

  protected:
    virtual void setStats(){}
};

class ScalarStatTester : public StatTester
{

  private:
    ScalarStatTesterParams params;

  public:
    ScalarStatTester(const ScalarStatTesterParams &p) :
        StatTester(p), params(p), stats(this, p) {}

  protected:
    void setStats() override;
    struct ScalarStatTesterStats : public statistics::Group
    {
        ScalarStatTesterStats(
            statistics::Group *parent,
            const ScalarStatTesterParams &params
        );
        statistics::Scalar scalar;
    } stats;
};

} // namespace gem5

#endif // __STAT_TESTER_HH__
