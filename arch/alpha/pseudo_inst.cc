/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#include <string>

#include "arch/alpha/pseudo_inst.hh"
#include "cpu/exec_context.hh"
#include "sim/param.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"

using namespace std;
using namespace Stats;

namespace AlphaPseudo
{
    bool doStatisticsInsts;
    bool doCheckpointInsts;
    bool doQuiesce;

    void
    arm(ExecContext *xc)
    {
        xc->kernelStats.arm();
    }

    void
    quiesce(ExecContext *xc)
    {
        if (!doQuiesce)
            return;

        xc->suspend();
        xc->kernelStats.quiesce();
    }

    void
    ivlb(ExecContext *xc)
    {
        xc->kernelStats.ivlb();
    }

    void
    ivle(ExecContext *xc)
    {
    }

    void
    m5exit_old(ExecContext *xc)
    {
        SimExit(curTick, "m5_exit_old instruction encountered");
    }

    void
    m5exit(ExecContext *xc)
    {
        Tick delay = xc->regs.intRegFile[16];
        Tick when = curTick + NS2Ticks(delay);
        SimExit(when, "m5_exit instruction encountered");
    }

    void
    resetstats(ExecContext *xc)
    {
        if (!doStatisticsInsts)
            return;

        Tick delay = xc->regs.intRegFile[16];
        Tick period = xc->regs.intRegFile[17];

        Tick when = curTick + NS2Ticks(delay);
        Tick repeat = NS2Ticks(period);

        using namespace Stats;
        SetupEvent(Reset, when, repeat);
    }

    void
    dumpstats(ExecContext *xc)
    {
        if (!doStatisticsInsts)
            return;

        Tick delay = xc->regs.intRegFile[16];
        Tick period = xc->regs.intRegFile[17];

        Tick when = curTick + NS2Ticks(delay);
        Tick repeat = NS2Ticks(period);

        using namespace Stats;
        SetupEvent(Dump, when, repeat);
    }

    void
    dumpresetstats(ExecContext *xc)
    {
        if (!doStatisticsInsts)
            return;

        Tick delay = xc->regs.intRegFile[16];
        Tick period = xc->regs.intRegFile[17];

        Tick when = curTick + NS2Ticks(delay);
        Tick repeat = NS2Ticks(period);

        using namespace Stats;
        SetupEvent(Dump|Reset, when, repeat);
    }

    void
    m5checkpoint(ExecContext *xc)
    {
        if (!doCheckpointInsts)
            return;

        Tick delay = xc->regs.intRegFile[16];
        Tick period = xc->regs.intRegFile[17];

        Tick when = curTick + NS2Ticks(delay);
        Tick repeat = NS2Ticks(period);

        Checkpoint::setup(when, repeat);
    }

    class Context : public ParamContext
    {
      public:
        Context(const string &section) : ParamContext(section) {}
        void checkParams();
    };

    Context context("PseudoInsts");

    Param<bool> __quiesce(&context, "quiesce",
                          "enable quiesce instructions",
                          true);
    Param<bool> __statistics(&context, "statistics",
                             "enable statistics pseudo instructions",
                             true);
    Param<bool> __checkpoint(&context, "checkpoint",
                             "enable checkpoint pseudo instructions",
                             true);

    void
    Context::checkParams()
    {
        doQuiesce = __quiesce;
        doStatisticsInsts = __statistics;
        doCheckpointInsts = __checkpoint;
    }
}
