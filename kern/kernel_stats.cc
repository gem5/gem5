/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <map>
#include <stack>
#include <string>

#include "arch/alpha/osfpal.hh"
#include "base/trace.hh"
#include "base/statistics.hh"
#include "base/stats/bin.hh"
#include "cpu/exec_context.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "kern/kernel_stats.hh"
#include "kern/linux/linux_syscalls.hh"

using namespace std;
using namespace Stats;

namespace Kernel {

const char *modestr[] = { "kernel", "user", "idle", "interrupt" };

Statistics::Statistics(ExecContext *context)
    : xc(context), idleProcess((Addr)-1), themode(kernel), lastModeTick(0),
      iplLast(0), iplLastTick(0)
{
    bin_int = xc->system->params->bin_int;
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

    _ivlb
        .name(name() + ".inst.ivlb")
        .desc("number of ivlb instructions executed")
        ;

    _ivle
        .name(name() + ".inst.ivle")
        .desc("number of ivle instructions executed")
        ;

    _hwrei
        .name(name() + ".inst.hwrei")
        .desc("number of hwrei instructions executed")
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

    _syscall
        .init(SystemCalls<Tru64>::Number)
        .name(name() + ".syscall")
        .desc("number of syscalls executed")
        .flags(total | pdf | nozero | nonan)
        ;

    for (int i = 0; i < SystemCalls<Tru64>::Number; ++i) {
        const char *str = SystemCalls<Tru64>::name(i);
        if (str) {
            _syscall.subname(i, str);
        }
    }

    _faults
        .init(Num_Faults)
        .name(name() + ".faults")
        .desc("number of faults")
        .flags(total | pdf | nozero | nonan)
        ;

    for (int i = 1; i < Num_Faults; ++i) {
        const char *str = FaultName(i);
        if (str)
            _faults.subname(i, str);
    }

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
Statistics::setIdleProcess(Addr idlepcbb)
{
    assert(themode == kernel || themode == interrupt);
    idleProcess = idlepcbb;
    themode = idle;
    changeMode(themode);
}

void
Statistics::changeMode(cpu_mode newmode)
{
    _mode[newmode]++;

    if (newmode == themode)
        return;

    DPRINTF(Context, "old mode=%-8s new mode=%-8s\n",
            modestr[themode], modestr[newmode]);

    _modeGood[newmode]++;
    _modeTicks[themode] += curTick - lastModeTick;

    xc->system->kernelBinning->changeMode(newmode);

    lastModeTick = curTick;
    themode = newmode;
}

void
Statistics::swpipl(int ipl)
{
    assert(ipl >= 0 && ipl <= 0x1f && "invalid IPL\n");

    _iplCount[ipl]++;

    if (ipl == iplLast)
        return;

    _iplGood[ipl]++;
    _iplTicks[iplLast] += curTick - iplLastTick;
    iplLastTick = curTick;
    iplLast = ipl;
}

void
Statistics::mode(cpu_mode newmode)
{
    Addr pcbb = xc->regs.ipr[AlphaISA::IPR_PALtemp23];

    if ((newmode == kernel || newmode == interrupt) &&
            pcbb == idleProcess)
        newmode = idle;

    if (bin_int == false && newmode == interrupt)
        newmode = kernel;

    changeMode(newmode);
}

void
Statistics::context(Addr oldpcbb, Addr newpcbb)
{
    assert(themode != user);

    _swap_context++;
    changeMode(newpcbb == idleProcess ? idle : kernel);
}

void
Statistics::callpal(int code)
{
    if (!PAL::name(code))
        return;

    _callpal[code]++;

    switch (code) {
      case PAL::callsys: {
          int number = xc->regs.intRegFile[0];
          if (SystemCalls<Tru64>::validSyscallNumber(number)) {
              int cvtnum = SystemCalls<Tru64>::convert(number);
              _syscall[cvtnum]++;
          }
      } break;

      case PAL::swpctx:
        if (xc->system->kernelBinning)
            xc->system->kernelBinning->palSwapContext(xc);
        break;
    }
}

void
Statistics::serialize(ostream &os)
{
    int exemode = themode;
    SERIALIZE_SCALAR(exemode);
    SERIALIZE_SCALAR(idleProcess);
    SERIALIZE_SCALAR(iplLast);
    SERIALIZE_SCALAR(iplLastTick);
    SERIALIZE_SCALAR(lastModeTick);
}

void
Statistics::unserialize(Checkpoint *cp, const string &section)
{
    int exemode;
    UNSERIALIZE_SCALAR(exemode);
    UNSERIALIZE_SCALAR(idleProcess);
    UNSERIALIZE_SCALAR(iplLast);
    UNSERIALIZE_SCALAR(iplLastTick);
    UNSERIALIZE_SCALAR(lastModeTick);
    themode = (cpu_mode)exemode;
}

/* end namespace Kernel */ }
