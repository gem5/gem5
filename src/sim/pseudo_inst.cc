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

#include <fstream>
#include <string>

#include "arch/vtophys.hh"
#include "base/annotate.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "cpu/quiesce_event.hh"
#include "arch/kernel_stats.hh"
#include "sim/pseudo_inst.hh"
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

namespace PseudoInst
{
    void
    arm(ThreadContext *tc)
    {
        if (tc->getKernelStats())
            tc->getKernelStats()->arm();
    }

    void
    quiesce(ThreadContext *tc)
    {
        if (!tc->getCpuPtr()->params->do_quiesce)
            return;

        DPRINTF(Quiesce, "%s: quiesce()\n", tc->getCpuPtr()->name());

        tc->suspend();
        if (tc->getKernelStats())
            tc->getKernelStats()->quiesce();
    }

    void
    quiesceNs(ThreadContext *tc, uint64_t ns)
    {
        if (!tc->getCpuPtr()->params->do_quiesce || ns == 0)
            return;

        EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

        Tick resume = curTick + Clock::Int::ns * ns;

        quiesceEvent->reschedule(resume, true);

        DPRINTF(Quiesce, "%s: quiesceNs(%d) until %d\n",
                tc->getCpuPtr()->name(), ns, resume);

        tc->suspend();
        if (tc->getKernelStats())
            tc->getKernelStats()->quiesce();
    }

    void
    quiesceCycles(ThreadContext *tc, uint64_t cycles)
    {
        if (!tc->getCpuPtr()->params->do_quiesce || cycles == 0)
            return;

        EndQuiesceEvent *quiesceEvent = tc->getQuiesceEvent();

        Tick resume = curTick + tc->getCpuPtr()->cycles(cycles);

        quiesceEvent->reschedule(resume, true);

        DPRINTF(Quiesce, "%s: quiesceCycles(%d) until %d\n",
                tc->getCpuPtr()->name(), cycles, resume);

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
        if (!tc->getCpuPtr()->params->do_statistics_insts)
            return;


        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        Stats::StatEvent(false, true, when, repeat);
    }

    void
    dumpstats(ThreadContext *tc, Tick delay, Tick period)
    {
        if (!tc->getCpuPtr()->params->do_statistics_insts)
            return;


        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        Stats::StatEvent(true, false, when, repeat);
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
        if (!tc->getCpuPtr()->params->do_statistics_insts)
            return;


        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        Stats::StatEvent(true, true, when, repeat);
    }

    void
    m5checkpoint(ThreadContext *tc, Tick delay, Tick period)
    {
        if (!tc->getCpuPtr()->params->do_checkpoint_insts)
            return;

        Tick when = curTick + delay * Clock::Int::ns;
        Tick repeat = period * Clock::Int::ns;

        schedExitSimLoop("checkpoint", when, repeat);
    }

    uint64_t
    readfile(ThreadContext *tc, Addr vaddr, uint64_t len, uint64_t offset)
    {
        const string &file = tc->getSystemPtr()->params()->readfile;
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

    void debugbreak(ThreadContext *tc)
    {
        debug_break();
    }

    void switchcpu(ThreadContext *tc)
    {
        exitSimLoop("switchcpu");
    }
}
