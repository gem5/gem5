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

#include <map>
#include <stack>
#include <string>

#include "arch/generic/linux/threadinfo.hh"
#include "arch/alpha/kernel_stats.hh"
#include "arch/alpha/osfpal.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Context.hh"
#include "kern/tru64/tru64_syscalls.hh"
#include "sim/system.hh"

using namespace std;
using namespace Stats;

namespace AlphaISA {
namespace Kernel {

const char *modestr[] = { "kernel", "user", "idle" };

Statistics::Statistics(System *system)
    : ::Kernel::Statistics(system),
      idleProcess((Addr)-1), themode(kernel), lastModeTick(0)
{
}

void
Statistics::regStats(const string &_name)
{
    ::Kernel::Statistics::regStats(_name);

    _callpal
        .init(256)
        .name(name() + ".callpal")
        .desc("number of callpals executed")
        .flags(total | pdf | nozero | nonan)
        ;

    for (int i = 0; i < PAL::NumCodes; ++i) {
        const char *str = PAL::name(i);
        if (str)
            _callpal.subname(i, str);
    }

    _hwrei
        .name(name() + ".inst.hwrei")
        .desc("number of hwrei instructions executed")
        ;

    _mode
        .init(cpu_mode_num)
        .name(name() + ".mode_switch")
        .desc("number of protection mode switches")
        ;

    for (int i = 0; i < cpu_mode_num; ++i)
        _mode.subname(i, modestr[i]);

    _modeGood
        .init(cpu_mode_num)
        .name(name() + ".mode_good")
        ;

    for (int i = 0; i < cpu_mode_num; ++i)
        _modeGood.subname(i, modestr[i]);

    _modeFraction
        .name(name() + ".mode_switch_good")
        .desc("fraction of useful protection mode switches")
        .flags(total)
        ;

    for (int i = 0; i < cpu_mode_num; ++i)
        _modeFraction.subname(i, modestr[i]);

    _modeFraction = _modeGood / _mode;

    _modeTicks
        .init(cpu_mode_num)
        .name(name() + ".mode_ticks")
        .desc("number of ticks spent at the given mode")
        .flags(pdf)
        ;
    for (int i = 0; i < cpu_mode_num; ++i)
        _modeTicks.subname(i, modestr[i]);

    _swap_context
        .name(name() + ".swap_context")
        .desc("number of times the context was actually changed")
        ;
}

void
Statistics::setIdleProcess(Addr idlepcbb, ThreadContext *tc)
{
    assert(themode == kernel);
    idleProcess = idlepcbb;
    themode = idle;
    changeMode(themode, tc);
}

void
Statistics::changeMode(cpu_mode newmode, ThreadContext *tc)
{
    _mode[newmode]++;

    if (newmode == themode)
        return;

    DPRINTF(Context, "old mode=%s new mode=%s pid=%d\n",
            modestr[themode], modestr[newmode],
            Linux::ThreadInfo(tc).curTaskPID());

    _modeGood[newmode]++;
    _modeTicks[themode] += curTick() - lastModeTick;

    lastModeTick = curTick();
    themode = newmode;
}

void
Statistics::mode(cpu_mode newmode, ThreadContext *tc)
{
    Addr pcbb = tc->readMiscRegNoEffect(IPR_PALtemp23);

    if (newmode == kernel && pcbb == idleProcess)
        newmode = idle;

    changeMode(newmode, tc);
}

void
Statistics::context(Addr oldpcbb, Addr newpcbb, ThreadContext *tc)
{
    assert(themode != user);

    _swap_context++;
    changeMode(newpcbb == idleProcess ? idle : kernel, tc);

    DPRINTF(Context, "Context Switch old pid=%d new pid=%d\n",
            Linux::ThreadInfo(tc, oldpcbb).curTaskPID(),
            Linux::ThreadInfo(tc, newpcbb).curTaskPID());
}

void
Statistics::callpal(int code, ThreadContext *tc)
{
    if (!PAL::name(code))
        return;

    _callpal[code]++;

    switch (code) {
      case PAL::callsys: {
          int number = tc->readIntReg(0);
          if (SystemCalls<Tru64>::validSyscallNumber(number)) {
              int cvtnum = SystemCalls<Tru64>::convert(number);
              _syscall[cvtnum]++;
          }
      } break;
    }
}

void
Statistics::serialize(CheckpointOut &cp) const
{
    ::Kernel::Statistics::serialize(cp);
    int exemode = themode;
    SERIALIZE_SCALAR(exemode);
    SERIALIZE_SCALAR(idleProcess);
    SERIALIZE_SCALAR(lastModeTick);
}

void
Statistics::unserialize(CheckpointIn &cp)
{
    ::Kernel::Statistics::unserialize(cp);
    int exemode;
    UNSERIALIZE_SCALAR(exemode);
    UNSERIALIZE_SCALAR(idleProcess);
    UNSERIALIZE_SCALAR(lastModeTick);
    themode = (cpu_mode)exemode;
}

} // namespace Kernel
} // namespace AlphaISA
