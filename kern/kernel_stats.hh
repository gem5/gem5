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
 */

#ifndef __KERNEL_STATS_HH__
#define __KERNEL_STATS_HH__

#include <map>
#include <stack>
#include <string>
#include <vector>

#include "cpu/static_inst.hh"

class BaseCPU;
class ExecContext;
class FnEvent;
// What does kernel stats expect is included?
class System;

namespace Kernel {

enum cpu_mode { kernel, user, idle, cpu_mode_num };
extern const char *modestr[];

class Statistics : public Serializable
{
  private:
    std::string myname;

    Addr idleProcess;
    cpu_mode themode;
    Tick lastModeTick;

    void changeMode(cpu_mode newmode, ExecContext *xc);

  private:
    Stats::Scalar<> _arm;
    Stats::Scalar<> _quiesce;
    Stats::Scalar<> _ivlb;
    Stats::Scalar<> _ivle;
    Stats::Scalar<> _hwrei;

    Stats::Vector<> _iplCount;
    Stats::Vector<> _iplGood;
    Stats::Vector<> _iplTicks;
    Stats::Formula _iplUsed;

    Stats::Vector<> _callpal;
    Stats::Vector<> _syscall;
//    Stats::Vector<> _faults;

    Stats::Vector<> _mode;
    Stats::Vector<> _modeGood;
    Stats::Formula _modeFraction;
    Stats::Vector<> _modeTicks;

    Stats::Scalar<> _swap_context;

  private:
    int iplLast;
    Tick iplLastTick;

  public:
    Statistics(System *system);

    const std::string name() const { return myname; }
    void regStats(const std::string &name);

  public:
    void arm() { _arm++; }
    void quiesce() { _quiesce++; }
    void ivlb() { _ivlb++; }
    void ivle() { _ivle++; }
    void hwrei() { _hwrei++; }
    void swpipl(int ipl);
    void mode(cpu_mode newmode, ExecContext *xc);
    void context(Addr oldpcbb, Addr newpcbb, ExecContext *xc);
    void callpal(int code, ExecContext *xc);

    void setIdleProcess(Addr idle, ExecContext *xc);

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

/* end namespace Kernel */ }

#endif // __KERNEL_STATS_HH__
