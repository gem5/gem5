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

#include "base/statistics.hh"
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "kern/kernel_stats.hh"
#include "sim/stats.hh"
#include "sim/sw_context.hh"
#include "targetarch/isa_traits.hh"
#include "targetarch/osfpal.hh"
#include "targetarch/syscalls.hh"

using namespace std;
using namespace Stats;

class KSData
{
  private:
    string _name;
    ExecContext *xc;
    BaseCPU *cpu;

  public:
    KSData(ExecContext *_xc, BaseCPU *_cpu)
        : xc(_xc), cpu(_cpu), iplLast(0), iplLastTick(0), lastUser(false),
          lastModeTick(0)
    {}

    const string &name() { return _name; }
    void regStats(const string &name);

  public:
    Scalar<> _arm;
    Scalar<> _quiesce;
    Scalar<> _ivlb;
    Scalar<> _ivle;
    Scalar<> _hwrei;

    Vector<> _iplCount;
    Vector<> _iplGood;
    Vector<> _iplTicks;
    Formula _iplUsed;

    Vector<> _callpal;
    Vector<> _syscall;
    Vector<> _faults;

    Vector<> _mode;
    Vector<> _modeGood;
    Formula _modeFraction;
    Vector<> _modeTicks;

    Scalar<> _swap_context;

  private:
    int iplLast;
    Tick iplLastTick;

    bool lastUser;
    Tick lastModeTick;

  public:
    void swpipl(int ipl);
    void mode(bool user);
    void callpal(int code);
};

KernelStats::KernelStats(ExecContext *xc, BaseCPU *cpu)
{ data = new KSData(xc, cpu); }

KernelStats::~KernelStats()
{ delete data; }

void
KernelStats::regStats(const string &name)
{ data->regStats(name); }

void
KSData::regStats(const string &name)
{
    _name = name;

    _arm
        .name(name + ".inst.arm")
        .desc("number of arm instructions executed")
        ;

    _quiesce
        .name(name + ".inst.quiesce")
        .desc("number of quiesce instructions executed")
        ;

    _ivlb
        .name(name + ".inst.ivlb")
        .desc("number of ivlb instructions executed")
        ;

    _ivle
        .name(name + ".inst.ivle")
        .desc("number of ivle instructions executed")
        ;

    _hwrei
        .name(name + ".inst.hwrei")
        .desc("number of hwrei instructions executed")
        ;

    _iplCount
        .init(32)
        .name(name + ".ipl_count")
        .desc("number of times we switched to this ipl")
        .flags(total | pdf | nozero | nonan)
        ;

    _iplGood
        .init(32)
        .name(name + ".ipl_good")
        .desc("number of times we switched to this ipl from a different ipl")
        .flags(total | pdf | nozero | nonan)
        ;

    _iplTicks
        .init(32)
        .name(name + ".ipl_ticks")
        .desc("number of cycles we spent at this ipl")
        .flags(total | pdf | nozero | nonan)
        ;

    _iplUsed
        .name(name + ".ipl_used")
        .desc("fraction of swpipl calls that actually changed the ipl")
        .flags(total | nozero | nonan)
        ;

    _iplUsed = _iplGood / _iplCount;

    _callpal
        .init(256)
        .name(name + ".callpal")
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
        .name(name + ".syscall")
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
        .name(name + ".faults")
        .desc("number of faults")
        .flags(total | pdf | nozero | nonan)
        ;

    for (int i = 1; i < Num_Faults; ++i) {
        const char *str = FaultName(i);
        if (str)
            _faults.subname(i, str);
    }

    _mode
        .init(2)
        .name(name + ".mode_switch")
        .subname(0, "kernel")
        .subname(1, "user")
        .desc("number of protection mode switches")
        ;

    _modeGood
        .init(2)
        .name(name + ".mode_good")
        ;

    _modeFraction
        .name(name + ".mode_switch_good")
        .subname(0, "kernel")
        .subname(1, "user")
        .desc("fraction of useful protection mode switches")
        .flags(total)
        ;
    _modeFraction = _modeGood / _mode;

    _modeTicks
        .init(2)
        .name(name + ".mode_ticks")
        .subname(0, "kernel")
        .subname(1, "user")
        .desc("number of ticks spent at the given mode")
        .flags(pdf)
        ;

    _swap_context
        .name(name + ".swap_context")
        .desc("number of times the context was actually changed")
        ;
}

void
KernelStats::arm()
{ data->_arm++; }

void
KernelStats::quiesce()
{ data->_quiesce++; }

void
KernelStats::ivlb()
{ data->_ivlb++; }

void
KernelStats::ivle()
{ data->_ivle++; }

void
KernelStats::hwrei()
{ data->_hwrei++; }

void
KernelStats::fault(Fault fault)
{ data->_faults[fault]++; }

void
KernelStats::swpipl(int ipl)
{ data->swpipl(ipl); }

void
KernelStats::mode(bool user)
{ data->mode(user); }

void
KernelStats::context(Addr old_pcbb, Addr new_pcbb)
{ data->_swap_context++; }

void
KernelStats::callpal(int code)
{ data->callpal(code); }


void
KSData::swpipl(int ipl)
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
KSData::mode(bool user)
{
    _mode[user]++;
    if (user == lastUser)
        return;

    _modeGood[user]++;
    _modeTicks[lastUser] += curTick - lastModeTick;

    lastModeTick = curTick;
    lastUser = user;

    if (xc->system->bin) {
        if (!xc->swCtx || xc->swCtx->callStack.empty()) {
            if (user)
                xc->system->User->activate();
            else
                xc->system->Kernel->activate();
        }
    }
}

void
KSData::callpal(int code)
{
    if (!PAL::name(code))
        return;

    _callpal[code]++;

    switch (code) {
      case PAL::callsys:
        {
            int number = xc->regs.intRegFile[0];
            if (SystemCalls<Tru64>::validSyscallNumber(number)) {
                int cvtnum = SystemCalls<Tru64>::convert(number);
                _syscall[cvtnum]++;
            }
        }
        break;
    }

    if (code == PAL::swpctx) {
        SWContext *out = xc->swCtx;
        System *sys = xc->system;
        if (!sys->bin)
            return;
        DPRINTF(TCPIP, "swpctx event\n");
        if (out) {
            DPRINTF(TCPIP, "swapping context out with this stack!\n");
            xc->system->dumpState(xc);
            Addr oldPCB = xc->regs.ipr[TheISA::IPR_PALtemp23];

            if (out->callStack.empty()) {
                DPRINTF(TCPIP, "but removing it, cuz empty!\n");
                SWContext *find = sys->findContext(oldPCB);
                if (find) {
                    assert(sys->findContext(oldPCB) == out);
                    sys->remContext(oldPCB);
                }
                delete out;
            } else {
                DPRINTF(TCPIP, "switching out context with pcb %#x, top fn %s\n",
                        oldPCB, out->callStack.top()->name);
                if (!sys->findContext(oldPCB)) {
                    if (!sys->addContext(oldPCB, out))
                        panic("could not add context");
                }
            }
        }

        Addr newPCB = xc->regs.intRegFile[16];
        SWContext *in = sys->findContext(newPCB);
        xc->swCtx = in;

        if (in) {
            assert(!in->callStack.empty() &&
                   "should not be switching in empty context");
            DPRINTF(TCPIP, "swapping context in with this callstack!\n");
            xc->system->dumpState(xc);
            sys->remContext(newPCB);
            fnCall *top = in->callStack.top();
            DPRINTF(TCPIP, "switching in to pcb %#x, %s\n", newPCB, top->name);
            assert(top->myBin && "should not switch to context with no Bin");
            top->myBin->activate();
        } else {
            sys->Kernel->activate();
        }
        DPRINTF(TCPIP, "end swpctx\n");
    }
}
