/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Lisa Hsu
 *          Nathan Binkert
 */

#include <string>

#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/kernel_stats.hh"
#if THE_ISA == ALPHA_ISA
#include "kern/tru64/tru64_syscalls.hh"
#endif
#include "sim/system.hh"

using namespace std;
using namespace Stats;

namespace Kernel {

Statistics::Statistics(System *system)
    : iplLast(0), iplLastTick(0)
{
}

void
Statistics::regStats(const string &_name)
{
    myname = _name;

    _arm
        .name(name() + ".inst.arm")
        .desc("number of arm instructions executed")
        ;

    _quiesce
        .name(name() + ".inst.quiesce")
        .desc("number of quiesce instructions executed")
        ;

    _iplCount
        .init(32)
        .name(name() + ".ipl_count")
        .desc("number of times we switched to this ipl")
        .flags(total | pdf | nozero | nonan)
        ;

    _iplGood
        .init(32)
        .name(name() + ".ipl_good")
        .desc("number of times we switched to this ipl from a different ipl")
        .flags(total | pdf | nozero | nonan)
        ;

    _iplTicks
        .init(32)
        .name(name() + ".ipl_ticks")
        .desc("number of cycles we spent at this ipl")
        .flags(total | pdf | nozero | nonan)
        ;

    _iplUsed
        .name(name() + ".ipl_used")
        .desc("fraction of swpipl calls that actually changed the ipl")
        .flags(total | nozero | nonan)
        ;

    _iplUsed = _iplGood / _iplCount;
#if THE_ISA == ALPHA_ISA
    _syscall
        .init(SystemCalls<Tru64>::Number)
        .name(name() + ".syscall")
        .desc("number of syscalls executed")
        .flags(total | pdf | nozero | nonan)
        ;
#endif

    //@todo This needs to get the names of syscalls from an appropriate place.
#if 0
    for (int i = 0; i < SystemCalls<Tru64>::Number; ++i) {
        const char *str = SystemCalls<Tru64>::name(i);
        if (str) {
            _syscall.subname(i, str);
        }
    }
#endif
}

void
Statistics::swpipl(int ipl)
{
    assert(ipl >= 0 && ipl <= 0x1f && "invalid IPL\n");

    _iplCount[ipl]++;

    if (ipl == iplLast)
        return;

    _iplGood[ipl]++;
    _iplTicks[iplLast] += curTick() - iplLastTick;
    iplLastTick = curTick();
    iplLast = ipl;
}

void
Statistics::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(iplLast);
    SERIALIZE_SCALAR(iplLastTick);
}

void
Statistics::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(iplLast);
    UNSERIALIZE_SCALAR(iplLastTick);
}

} // namespace Kernel
