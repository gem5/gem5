/*
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>

#include "sim/pseudo_inst.hh"
#include "arch/vtophys.hh"
#include "base/annotate.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "cpu/quiesce_event.hh"
#include "kern/kernel_stats.hh"
#include "sim/param.hh"
#include "sim/serialize.hh"
#include "sim/sim_exit.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/system.hh"
#include "sim/debug.hh"
#include "sim/vptr.hh"

using namespace std;

using namespace Stats;
using namespace TheISA;

namespace AlphaPseudo
{
    bool doStatisticsInsts;
    bool doCheckpointInsts;
    bool doQuiesce;

    void
    arm(ThreadContext *tc)
    {
        if (tc->getKernelStats())
            tc->getKernelStats()->arm();
    }

    void
    quiesce(ThreadContext *tc)
    {
        if (!doQuiesce)
            return;

        tc->suspend();
        if (tc->getKernelStats())
            tc->getKernelStats()->quiesce();
    }

    void
    quiesceNs(ThreadContext *tc, uint64_t ns)
    {
        if (!doQuiesce || ns == 0)
            return;

        EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

        if (quiesceEvent->scheduled())
            quiesceEvent->reschedule(curTick + Clock::Int::ns * ns);
        else
            quiesceEvent->schedule(curTick + Clock::Int::ns * ns);

        tc->suspend();
        if (tc->getKernelStats())
            tc->getKernelStats()->quiesce();
    }

    void
    quiesceCycles(ThreadContext *tc, uint64_t cycles)
    {
        if (!doQuiesce || cycles == 0)
            return;

        EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

        if (quiesceEvent->scheduled())
            quiesceEvent->reschedule(curTick +
                                     tc->getCpuPtr()->cycles(cycles));
        else
            quiesceEvent->schedule(curTick +
                                   tc->getCpuPtr()->cycles(cycles));

        tc->suspend();
        if (tc->getKernelStats())
            tc->getKernelStats()->quiesce();
    }

    uint64_t
    quiesceTime(ThreadContext *tc)
    {
        return (tc->readLastActivate() - tc->readLastSuspend()) / Clock::Int::ns;
    }

    void
    ivlb(ThreadContext *tc)
    {
        if (tc->getKernelStats())
            tc->getKernelStats()->ivlb();
    }

    void
    ivle(ThreadContext *tc)
    {
    }

    void
    m5exit_old(ThreadContext *tc)
    {
        exitSimLoop("m5_exit_old instruction encountered");
    }

    void
    m5exit(ThreadContext *tc, Tick delay)
    {
        Tick when = curTick + delay * Clock::Int::ns;
        schedExitSimLoop("m5_exit instruction encountered", when);
    }

    void
    loadsymbol(ThreadContext *tc)
    {
        const string &filename = tc->getCpuPtr()->system->params()->symbolfile;
        if (filename.empty()) {
            return;
        }

        std::string buffer;
        ifstream file(filename.c_str());

        if (!file)
            fatal("file error: Can't open symbol table file %s\n", filename);

        while (!file.eof()) {
            getline(file, buffer);

            if (buffer.empty())
                continue;

            int idx = buffer.find(' ');
            if (idx == string::npos)
                continue;

            string address = "0x" + buffer.substr(0, idx);
            eat_white(address);
            if (address.empty())
                continue;

            // Skip over letter and space
            string symbol = buffer.substr(idx + 3);
            eat_white(symbol);
            if (symbol.empty())
                continue;

            Addr addr;
            if (!to_number(address, addr))
                continue;

            if (!tc->getSystemPtr()->kernelSymtab->insert(addr, symbol))
                continue;


            DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);
        }
        file.close();
    }

    void
    resetstats(ThreadContext *tc, Tick delay, Tick period)
    {
        if (!doStatisticsInsts)
            return;


        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        using namespace Stats;
        SetupEvent(Reset, when, repeat);
    }

    void
    dumpstats(ThreadContext *tc, Tick delay, Tick period)
    {
        if (!doStatisticsInsts)
            return;


        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        using namespace Stats;
        SetupEvent(Dump, when, repeat);
    }

    void
    addsymbol(ThreadContext *tc, Addr addr, Addr symbolAddr)
    {
        char symb[100];
        CopyStringOut(tc, symb, symbolAddr, 100);
        std::string symbol(symb);

        DPRINTF(Loader, "Loaded symbol: %s @ %#llx\n", symbol, addr);

        tc->getSystemPtr()->kernelSymtab->insert(addr,symbol);
    }

    void
    anBegin(ThreadContext *tc, uint64_t cur)
    {
        Annotate::annotations.add(tc->getSystemPtr(), 0, cur >> 32, cur &
                0xFFFFFFFF, 0,0);
    }

    void
    anWait(ThreadContext *tc, uint64_t cur, uint64_t wait)
    {
        Annotate::annotations.add(tc->getSystemPtr(), 0, cur >> 32, cur &
                0xFFFFFFFF, wait >> 32, wait & 0xFFFFFFFF);
    }


    void
    dumpresetstats(ThreadContext *tc, Tick delay, Tick period)
    {
        if (!doStatisticsInsts)
            return;


        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        using namespace Stats;
        SetupEvent(Dump|Reset, when, repeat);
    }

    void
    m5checkpoint(ThreadContext *tc, Tick delay, Tick period)
    {
        if (!doCheckpointInsts)
            return;

        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        schedExitSimLoop("checkpoint", when, repeat);
    }

    uint64_t
    readfile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset)
    {
        const string &file = tc->getCpuPtr()->system->params()->readfile;
        if (file.empty()) {
            return ULL(0);
        }

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
        CopyIn(tc, vaddr, buf, result);
        delete [] buf;
        return result;
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

    void debugbreak(ThreadContext *tc)
    {
        debug_break();
    }

    void switchcpu(ThreadContext *tc)
    {
        exitSimLoop("switchcpu");
    }
}
