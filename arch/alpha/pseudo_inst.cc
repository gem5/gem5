/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

#include <string>

#include "arch/alpha/pseudo_inst.hh"
#include "arch/alpha/vtophys.hh"
#include "cpu/base.hh"
#include "cpu/sampler/sampler.hh"
#include "cpu/exec_context.hh"
#include "kern/kernel_stats.hh"
#include "sim/param.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/system.hh"
#include "sim/debug.hh"
#include "targetarch/vptr.hh"

using namespace std;

extern Sampler *SampCPU;

using namespace Stats;

namespace AlphaPseudo
{
    bool doStatisticsInsts;
    bool doCheckpointInsts;
    bool doQuiesce;

    void
    arm(ExecContext *xc)
    {
        xc->kernelStats->arm();
    }

    void
    quiesce(ExecContext *xc)
    {
        if (!doQuiesce)
            return;

        xc->suspend();
        xc->kernelStats->quiesce();
    }

    void
    ivlb(ExecContext *xc)
    {
        xc->kernelStats->ivlb();
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
        Tick when = curTick + delay * Clock::Int::ns;
        SimExit(when, "m5_exit instruction encountered");
    }

    void
    resetstats(ExecContext *xc)
    {
        if (!doStatisticsInsts)
            return;

        Tick delay = xc->regs.intRegFile[16];
        Tick period = xc->regs.intRegFile[17];

        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

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

        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        using namespace Stats;
        SetupEvent(Dump, when, repeat);
    }

    void
    addsymbol(ExecContext *xc)
    {
        Addr addr = xc->regs.intRegFile[16];
        char symb[100];
        CopyString(xc, symb, xc->regs.intRegFile[17], 100);
        std::string symbol(symb);

        DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);

        xc->system->kernelSymtab->insert(addr,symbol);
    }

    void
    dumpresetstats(ExecContext *xc)
    {
        if (!doStatisticsInsts)
            return;

        Tick delay = xc->regs.intRegFile[16];
        Tick period = xc->regs.intRegFile[17];

        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

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

        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        Checkpoint::setup(when, repeat);
    }

    void
    readfile(ExecContext *xc)
    {
        const string &file = xc->cpu->system->params->readfile;
        if (file.empty()) {
            xc->regs.intRegFile[0] = ULL(0);
            return;
        }

        Addr vaddr = xc->regs.intRegFile[16];
        uint64_t len = xc->regs.intRegFile[17];
        uint64_t offset = xc->regs.intRegFile[18];
        uint64_t result = 0;

        int fd = ::open(file.c_str(), O_RDONLY, 0);
        if (fd < 0)
            panic("could not open file %s\n", file);

        if (::lseek(fd, offset, SEEK_SET) < 0)
            panic("could not seek: %s", strerror(errno));

        char *buf = new char[len];
        char *p = buf;
        while (len > 0) {
            int bytes = ::read(fd, p, len);
            if (bytes <= 0)
                break;

            p += bytes;
            result += bytes;
            len -= bytes;
        }

        close(fd);
        CopyIn(xc, vaddr, buf, result);
        delete [] buf;
        xc->regs.intRegFile[0] = result;
    }

    class Context : public ParamContext
    {
      public:
        Context(const string &section) : ParamContext(section) {}
        void checkParams();
    };

    Context context("pseudo_inst");

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

    void debugbreak(ExecContext *xc)
    {
        debug_break();
    }

    void switchcpu(ExecContext *xc)
    {
        if (SampCPU)
            SampCPU->switchCPUs();
    }
}
