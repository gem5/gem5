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

#ifndef __KERNEL_STATS_HH__
#define __KERNEL_STATS_HH__

#include <map>
#include <stack>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "sim/serialize.hh"
#include "targetarch/isa_traits.hh"

class BaseCPU;
class ExecContext;
class FnEvent;
enum Fault;

namespace Kernel {

enum cpu_mode { kernel, user, idle, cpu_mode_num };
extern const char *modestr[];

class Binning
{
  private:
    std::string myname;
    System *system;

  private:
    // lisa's binning stuff
    struct fnCall
    {
        Stats::MainBin *myBin;
        std::string name;
    };

    struct SWContext
    {
        Counter calls;
        std::stack<fnCall *> callStack;
    };

    std::map<const std::string, Stats::MainBin *> fnBins;
    std::map<const Addr, SWContext *> swCtxMap;

    std::multimap<const std::string, std::string> callerMap;
    void populateMap(std::string caller, std::string callee);

    std::vector<FnEvent *> fnEvents;

    Stats::Scalar<> fnCalls;

    Stats::MainBin *getBin(const std::string &name);
    bool findCaller(std::string, std::string) const;

    SWContext *findContext(Addr pcb);
    bool addContext(Addr pcb, SWContext *ctx)
    {
        return (swCtxMap.insert(std::make_pair(pcb, ctx))).second;
    }

    void remContext(Addr pcb)
    {
        swCtxMap.erase(pcb);
    }

    void dumpState() const;

    SWContext *swctx;
    std::vector<std::string> binned_fns;

  private:
    Stats::MainBin *modeBin[3];

  public:
    const bool bin;
    const bool fnbin;

    cpu_mode themode;
    void palSwapContext(ExecContext *xc);
    void execute(ExecContext *xc, const StaticInstBase *inst);
    void call(ExecContext *xc, Stats::MainBin *myBin);
    void changeMode(cpu_mode mode);

  public:
    Binning(System *sys);
    virtual ~Binning();

    const std::string name() const { return myname; }
    void regStats(const std::string &name);

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

class Statistics : public Serializable
{
    friend class Binning;

  private:
    std::string myname;
    ExecContext *xc;

    Addr idleProcess;
    cpu_mode themode;
    Tick lastModeTick;

    void changeMode(cpu_mode newmode);

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
    Stats::Vector<> _faults;

    Stats::Vector<> _mode;
    Stats::Vector<> _modeGood;
    Stats::Formula _modeFraction;
    Stats::Vector<> _modeTicks;

    Stats::Scalar<> _swap_context;

  private:
    int iplLast;
    Tick iplLastTick;

  public:
    Statistics(ExecContext *context);

    const std::string name() const { return myname; }
    void regStats(const std::string &name);

  public:
    void arm() { _arm++; }
    void quiesce() { _quiesce++; }
    void ivlb() { _ivlb++; }
    void ivle() { _ivle++; }
    void hwrei() { _hwrei++; }
    void fault(Fault fault) { _faults[fault]++; }
    void swpipl(int ipl);
    void mode(bool usermode);
    void context(Addr oldpcbb, Addr newpcbb);
    void callpal(int code);

    void setIdleProcess(Addr idle);

  public:
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

/* end namespace Kernel */ }

#endif // __KERNEL_STATS_HH__
