/*
 * Copyright (c) 2006-2009 The Regents of The University of Michigan
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

#include "base/cp_annotate.hh"

#include "arch/generic/linux/threadinfo.hh"
#include "arch/utility.hh"
#include "base/callback.hh"
#include "base/loader/object_file.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "debug/Annotate.hh"
#include "debug/AnnotateVerbose.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

struct CPAIgnoreSymbol
{
    const char *symbol;
    size_t len;
};
#define CPA_IGNORE_SYMBOL(sym) { #sym, sizeof(#sym) }

CPAIgnoreSymbol ignoreSymbols[] = {
    CPA_IGNORE_SYMBOL("m5a_"),
    CPA_IGNORE_SYMBOL("ret_from_sys_call"),
    CPA_IGNORE_SYMBOL("ret_from_reschedule"),
    CPA_IGNORE_SYMBOL("_spin_"),
    CPA_IGNORE_SYMBOL("local_bh_"),
    CPA_IGNORE_SYMBOL("restore_all"),
    CPA_IGNORE_SYMBOL("Call_Pal_"),
    CPA_IGNORE_SYMBOL("pal_post_interrupt"),
    CPA_IGNORE_SYMBOL("rti_to_"),
    CPA_IGNORE_SYMBOL("sys_int_2"),
    CPA_IGNORE_SYMBOL("sys_interrupt"),
    CPA_IGNORE_SYMBOL("normal_int"),
    CPA_IGNORE_SYMBOL("TRAP_INTERRUPT_10_"),
    CPA_IGNORE_SYMBOL("Trap_Interrupt"),
    CPA_IGNORE_SYMBOL("do_entInt"),
    CPA_IGNORE_SYMBOL("__do_softirq"),
    CPA_IGNORE_SYMBOL("_end"),
    CPA_IGNORE_SYMBOL("entInt"),
    CPA_IGNORE_SYMBOL("entSys"),
    {0,0}
};
#undef CPA_IGNORE_SYMBOL

using namespace std;
using namespace TheISA;

bool CPA::exists;
CPA *CPA::_cpa;

class AnnotateDumpCallback : public Callback
{

  private:
    CPA *cpa;
  public:
    virtual void process();
    AnnotateDumpCallback(CPA *_cpa)
        : cpa(_cpa)
    {}
};

void
AnnotateDumpCallback::process()
{
    cpa->dump(true);
    cpa->dumpKey();
}


CPA::CPA(Params *p)
    : SimObject(p), numSm(0), numSmt(0), numSys(0), numQs(0), conId(0)
{
    if (exists)
        fatal("Multiple annotation objects found in system");
    exists = true;

    _enabled = p->enabled;
    _cpa = this;

    vector<string>::iterator i;
    i = p->user_apps.begin();

    while (i != p->user_apps.end()) {
        auto *of = createObjectFile(*i);
        string sf;
        if (!of)
            fatal("Couldn't load symbols from file: %s\n", *i);
        sf = *i;
        sf.erase(0, sf.rfind('/') + 1);;
        DPRINTFN("file %s short: %s\n", *i, sf);
        userApp[sf] = new Loader::SymbolTable;
        bool result1 = of->loadGlobalSymbols(userApp[sf]);
        bool result2 = of->loadLocalSymbols(userApp[sf]);
        if (!result1 || !result2)
            panic("blah");
        assert(result1 && result2);
        i++;
    }
}

void
CPA::startup()
{
    osbin = simout.create("annotate.bin", true);
    // MAGIC version number 'M''5''A'N' + version/capabilities
    ah.version = 0x4D35414E00000101ULL;
    ah.num_recs = 0;
    ah.key_off = 0;
    osbin->write((char*)&ah, sizeof(AnnotateHeader));

    registerExitCallback(new AnnotateDumpCallback(this));
}

uint64_t
CPA::getFrame(ThreadContext *tc)
{
    // This code is ISA specific and will need to be changed
    // if the annotation code is used for something other than Alpha
    return (tc->readMiscRegNoEffect(TheISA::IPR_PALtemp23) &
            ~ULL(0x3FFF));

}

void
CPA::swSmBegin(ThreadContext *tc, Addr sm_string, int32_t sm_id, int32_t flags)
{
    if (!enabled())
        return;

    std::string st;
    Addr junk;
    char sm[50];
    if (!TheISA::inUserMode(tc))
        Loader::debugSymbolTable->findNearestSymbol(
            tc->readIntReg(ReturnAddressReg), st, junk);

    tc->getVirtProxy().readString(sm, sm_string, 50);
    System *sys = tc->getSystemPtr();
    StringWrap name(sys->name());

    if (!sm[0])
        warn("Got null SM at tick %d\n", curTick());

    int sysi = getSys(sys);
    int smi = getSm(sysi, sm, sm_id);
    DPRINTF(Annotate,  "Starting machine: %s(%d) sysi: %d id: %#x\n", sm,
            smi, sysi, sm_id);
    DPRINTF(Annotate, "smMap[%d] = %d, %s, %#x\n", smi,
            smMap[smi-1].first, smMap[smi-1].second.first,
            smMap[smi-1].second.second);

    uint64_t frame = getFrame(tc);
    StackId sid = StackId(sysi, frame);

    // check if we need to link to the previous state machine
    if (flags & FL_LINK) {
        if (smStack[sid].size()) {
            int prev_smi = smStack[sid].back();
            DPRINTF(Annotate, "Linking from %d to state machine %s(%d) [%#x]\n",
                    prev_smi, sm, smi, sm_id);

            if (lnMap[smi])
                DPRINTF(Annotate, "LnMap already contains entry for %d of %d\n",
                        smi, lnMap[smi]);
            assert(lnMap[smi] == 0);
            lnMap[smi] =  prev_smi;

            add(OP_LINK, FL_NONE, tc->contextId(), prev_smi, smi);
        } else {
            DPRINTF(Annotate, "Not Linking to state machine %s(%d) [%#x]\n",
                    sm, smi, sm_id);
        }
    }


    smStack[sid].push_back(smi);

    DPRINTF(Annotate, "Stack Now (%#X):\n", frame);
    for (int x = smStack[sid].size()-1; x >= 0; x--)
        DPRINTF(Annotate, "-- %d\n", smStack[sid][x]);

    // reset the sw state exculsion to false
    if (swExpl[sid])
        swExpl[sid] = false;


    Id id = Id(sm, frame);
    if (scLinks[sysi-1][id]) {
        AnnDataPtr an = scLinks[sysi-1][id];
        scLinks[sysi-1].erase(id);
        an->stq = smi;
        an->dump = true;
        DPRINTF(Annotate,
                "Found prev unknown linking from %d to state machine %s(%d)\n",
                an->sm, sm, smi);

        if (lnMap[smi])
            DPRINTF(Annotate, "LnMap already contains entry for %d of %d\n",
                    smi, lnMap[smi]);
        assert(lnMap[smi] == 0);
        lnMap[smi] =  an->sm;
    }

    // add a new begin ifwe have that info
    if (st != "") {
        DPRINTF(Annotate, "st: %s smi: %d stCache.size %d\n", st,
                smi, stCache.size());
        int sti = getSt(sm, st);
        lastState[smi] = sti;
        add(OP_BEGIN, FL_NONE, tc->contextId(), smi, sti);
    }
}

void
CPA::swSmEnd(ThreadContext *tc, Addr sm_string)
{
    if (!enabled())
        return;

    char sm[50];
    tc->getVirtProxy().readString(sm, sm_string, 50);
    System *sys = tc->getSystemPtr();
    doSwSmEnd(sys, tc->contextId(), sm, getFrame(tc));
}

void
CPA::doSwSmEnd(System *sys, int cpuid, string sm, uint64_t frame)
{
    int sysi = getSys(sys);
    StackId sid = StackId(sysi, frame);


    // reset the sw state exculsion to false
    if (swExpl[sid])
        swExpl[sid] = false;


    int smib = smStack[sid].back();
    StringWrap name(sys->name());
    DPRINTF(Annotate, "Ending machine: %s[%d, %#x] (%d?)\n", sm, sysi,
            frame, smib);

    if (!smStack[sid].size() || smMap[smib-1].second.first != sm) {
        DPRINTF(Annotate, "State Machine not unwinding correctly. sid: %d, %#x"
                " top of stack: %s Current Stack:\n",
                sysi, frame, smMap[smib-1].second.first);
        for (int x = smStack[sid].size()-1; x >= 0; x--)
            DPRINTF(Annotate, "-- %d\n", smStack[sid][x]);
        DPRINTF(Annotate, "Ending machine: %s; end stack: %s\n", sm,
                smMap[smib-1].second.first);

        warn("State machine stack not unwinding correctly at %d\n", curTick());
    } else {
        DPRINTF(Annotate,
                "State machine ending:%s sysi:%d id:%#x back:%d getSm:%d\n",
                sm, sysi, smMap[smib-1].second.second, smStack[sid].back(),
                getSm(sysi, sm, smMap[smib-1].second.second));
        assert(getSm(sysi, sm, smMap[smib-1].second.second) ==
                smStack[sid].back());

        int smi = smStack[sid].back();
        smStack[sid].pop_back();

        if (lnMap[smi]) {
            DPRINTF(Annotate, "Linking %d back to %d\n", smi, lnMap[smi]);
            add(OP_LINK, FL_NONE, cpuid, smi, lnMap[smi]);
            lnMap.erase(smi);
        }

        if (smStack[sid].size()) {
            add(OP_BEGIN, FL_NONE, cpuid, smi, lastState[smi]);
        }

        DPRINTF(Annotate, "Stack Now:\n");
        for (int x = smStack[sid].size()-1; x >= 0; x--)
            DPRINTF(Annotate, "-- %d\n", smStack[sid][x]);
    }
}


void
CPA::swExplictBegin(ThreadContext *tc, int32_t flags, Addr st_string)
{
    if (!enabled())
        return;

    char st[50];
    tc->getVirtProxy().readString(st, st_string, 50);

    StringWrap name(tc->getSystemPtr()->name());
    DPRINTF(Annotate, "Explict begin of state %s\n", st);
    if (flags & FL_BAD)
        warn("BAD state encountered: at cycle %d: %s\n", curTick(), st);
    swBegin(tc->getSystemPtr(), tc->contextId(),
            st, getFrame(tc), true, flags);
}

void
CPA::swAutoBegin(ThreadContext *tc, Addr next_pc)
{
    if (!enabled())
        return;

    string sym;
    Addr sym_addr = 0;

    if (!TheISA::inUserMode(tc)) {
        Loader::debugSymbolTable->findNearestSymbol(next_pc, sym, sym_addr);
    } else {
        Linux::ThreadInfo ti(tc);
        string app = ti.curTaskName();
        if (userApp.count(app))
            userApp[app]->findNearestSymbol(next_pc, sym, sym_addr);
    }

    if (sym_addr)
        swBegin(tc->getSystemPtr(), tc->contextId(), sym, getFrame(tc));
}

void
CPA::swBegin(System *sys, int cpuid, std::string st, uint64_t frame, bool expl,
        int flags)
{
    int x = 0;
    int len;
    while (ignoreSymbols[x].len)
    {
        len = ignoreSymbols[x].len;
        if (!st.compare(0,len, ignoreSymbols[x].symbol, len))
            return;
        x++;
    }

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, frame);
    // if expl is true suspend symbol table based states
    if (!smStack[sid].size())
        return;
    if (!expl && swExpl[sid])
        return;
    if (expl)
        swExpl[sid] = true;
    DPRINTFS(AnnotateVerbose, sys, "SwBegin: %s sysi: %d\n", st, sysi);
    int smi = smStack[sid].back();
    int sti = getSt(smMap[smi-1].second.first, st);
    if (lastState[smi] != sti) {
        lastState[smi] = sti;
        add(OP_BEGIN, flags, cpuid, smi, sti);
    }
}

void
CPA::swEnd(ThreadContext *tc)
{
    if (!enabled())
        return;

    std::string st;
    Addr junk;
    if (!TheISA::inUserMode(tc))
        Loader::debugSymbolTable->findNearestSymbol(
            tc->readIntReg(ReturnAddressReg), st, junk);
    System *sys = tc->getSystemPtr();
    StringWrap name(sys->name());

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size()) {
        DPRINTF(Annotate, "Explict end of State: %s IGNORED\n",  st);
        return;
    }
    DPRINTF(Annotate, "Explict end of State: %s\n",  st);
    // return back to symbol table based states
    swExpl[sid] = false;
    int smi = smStack[sid].back();
    if (st != "") {
        int sti = getSt(smMap[smi-1].second.first, st);
        lastState[smi] = sti;
        add(OP_BEGIN, FL_NONE, tc->contextId(), smi, sti);
    }
}

void
CPA::swQ(ThreadContext *tc, Addr id, Addr q_string, int32_t count)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    if (swExpl[sid])
        swExpl[sid] = false;
    int qi = getQ(sysi, q, id);
    if (count == 0) {
        //warn("Tried to queue 0 bytes in %s, ignoring\n", q);
        return;
    }
    DPRINTFS(AnnotateQ, sys,
            "swQ: %s[%#x] cur size %d %d bytes: %d adding: %d\n",
            q, id, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);
    doQ(sys, FL_NONE, tc->contextId(), smi, q, qi, count);
}

void
CPA::swDq(ThreadContext *tc, Addr id, Addr q_string, int32_t count)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int qi = getQ(sysi, q, id);
    if (swExpl[sid])
        swExpl[sid] = false;
    DPRINTFS(AnnotateQ, sys,
            "swDq: %s[%#x] cur size %d %d bytes: %d removing: %d\n",
            q, id, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);
    assert(count != 0);

    doDq(sys, FL_NONE, tc->contextId(), smi, q, qi, count);
}

void
CPA::swPq(ThreadContext *tc, Addr id, Addr q_string, int32_t count)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int qi = getQ(sysi, q, id);
    if (swExpl[sid])
        swExpl[sid] = false;
    DPRINTFS(AnnotateQ, sys,
            "swPq: %s [%#x] cur size %d %d bytes: %d peeking: %d\n",
            q, id, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);

    assert(count != 0);
    if (qBytes[qi-1] < count) {
        dump(true);
        dumpKey();
        fatal("Queue %s peeking with not enough bytes available in queue!\n", q);
    }

    add(OP_PEEK, FL_NONE, tc->contextId(), smi, qi, count);
}

void
CPA::swRq(ThreadContext *tc, Addr id, Addr q_string, int32_t count)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int qi = getQ(sysi, q, id);
    if (swExpl[sid])
        swExpl[sid] = false;
    DPRINTFS(AnnotateQ, sys,
            "swRq: %s [%#x] cur size %d %d bytes: %d reserve: %d\n",
            q, id, qSize[qi-1], qData[qi-1].size(), qBytes[qi-1], count);

    assert(count != 0);

    add(OP_RESERVE, FL_NONE, tc->contextId(), smi, qi, count);
}


void
CPA::swWf(ThreadContext *tc, Addr id, Addr q_string, Addr sm_string,
        int32_t count)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int qi = getQ(sysi, q, id);
    add(OP_WAIT_FULL, FL_NONE, tc->contextId(), smi, qi, count);

    if (!!sm_string) {
        char sm[50];
        tc->getVirtProxy().readString(sm, sm_string, 50);
        doSwSmEnd(tc->getSystemPtr(), tc->contextId(), sm, getFrame(tc));
    }
}

void
CPA::swWe(ThreadContext *tc, Addr id, Addr q_string, Addr sm_string,
        int32_t count)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int qi = getQ(sysi, q, id);
    add(OP_WAIT_EMPTY, FL_NONE, tc->contextId(), smi, qi, count);

    if (!!sm_string) {
        char sm[50];
        tc->getVirtProxy().readString(sm, sm_string, 50);
        doSwSmEnd(tc->getSystemPtr(), tc->contextId(), sm, getFrame(tc));
    }
}

void
CPA::swSq(ThreadContext *tc, Addr id, Addr q_string, int32_t size,
        int32_t flags)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();
    StringWrap name(sys->name());

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int qi = getQ(sysi, q, id);
    DPRINTF(AnnotateQ, "swSq: %s [%#x] cur size: %d bytes: %d, new size: %d\n",
             q, id, qSize[qi-1], qBytes[qi-1], size);

    if (FL_RESET & flags) {
        DPRINTF(AnnotateQ, "Resetting Queue %s\n", q);
        add(OP_SIZE_QUEUE, FL_NONE, tc->contextId(), smi, qi, 0);
        qData[qi-1].clear();
        qSize[qi-1] = 0;
        qBytes[qi-1] = 0;
    }

    if (qBytes[qi-1] < size)
        doQ(sys, FL_NONE, tc->contextId(), smi, q, qi, size - qBytes[qi-1]);
    else if (qBytes[qi-1] > size) {
        DPRINTF(AnnotateQ, "removing for resize of queue %s\n", q);
        add(OP_SIZE_QUEUE, FL_NONE, tc->contextId(), smi, qi, size);
        if (size <= 0) {
            qData[qi-1].clear();
            qSize[qi-1] = 0;
            qBytes[qi-1] = 0;
            return;
        }
        int need = qBytes[qi-1] - size;
        qBytes[qi-1] = size;
        while (need > 0) {
            int32_t tail_bytes = qData[qi-1].back()->data;
            if (qSize[qi-1] <= 0 || qBytes[qi-1] < 0) {
                dump(true);
                dumpKey();
                fatal("Queue %s had inconsistancy when doing size queue!\n", q);
            }
            if (tail_bytes > need) {
                qData[qi-1].back()->data -= need;
                need = 0;
            } else if (tail_bytes == need) {
                qData[qi-1].pop_back();
                qSize[qi-1]--;
                need = 0;
            } else {
                qData[qi-1].pop_back();
                qSize[qi-1]--;
                need -= tail_bytes;
            }
        }
    }
}

void
CPA::swAq(ThreadContext *tc, Addr id, Addr q_string, int32_t size)
{
    if (!enabled())
        return;

    char q[50];
    tc->getVirtProxy().readString(q, q_string, 50);
    System *sys = tc->getSystemPtr();
    StringWrap name(sys->name());

    int sysi = getSys(sys);
    int qi = getQ(sysi, q, id);
    if (qBytes[qi-1] != size) {
        DPRINTF(AnnotateQ, "Queue %s [%#x] has inconsintant size\n", q, id);
        //dump(true);
        //dumpKey();
        std::list<AnnDataPtr>::iterator ai = qData[qi-1].begin();
        int x = 0;
        while (ai != qData[qi-1].end()) {
            DPRINTF(AnnotateQ, "--Element %d size %d\n", x, (*ai)->data);
            ai++;
            x++;
        }

        warn("%d: Queue Assert: SW said there should be %d byte(s) in %s,"
                "however there are %d byte(s)\n",
            curTick(), size, q, qBytes[qi-1]);
        DPRINTF(AnnotateQ, "%d: Queue Assert: SW said there should be %d"
                " byte(s) in %s, however there are %d byte(s)\n",
            curTick(), size, q, qBytes[qi-1]);
    }
}

void
CPA::swLink(ThreadContext *tc, Addr lsm_string, Addr lsm_id, Addr sm_string)
{
    if (!enabled())
        return;

    char lsm[50];
    tc->getVirtProxy().readString(lsm, lsm_string, 50);
    System *sys = tc->getSystemPtr();
    StringWrap name(sys->name());

    int sysi = getSys(sys);
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();
    int lsmi = getSm(sysi, lsm, lsm_id);

    DPRINTF(Annotate, "Linking from %d to state machine %s(%d) [%#x]\n",
            smi, lsm, lsmi, lsm_id);

    if (lnMap[lsmi])
        DPRINTF(Annotate, "LnMap already contains entry for %d of %d\n",
                lsmi, lnMap[lsmi]);
    assert(lnMap[lsmi] == 0);
    lnMap[lsmi] =  smi;

    add(OP_LINK, FL_NONE, tc->contextId(), smi, lsmi);

    if (!!sm_string) {
        char sm[50];
        tc->getVirtProxy().readString(sm, sm_string, 50);
        doSwSmEnd(tc->getSystemPtr(), tc->contextId(), sm, getFrame(tc));
    }
}

void
CPA::swIdentify(ThreadContext *tc, Addr smi_string)
{
    if (!enabled())
        return;

    int sysi = getSys(tc->getSystemPtr());
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        return;
    int smi = smStack[sid].back();

    DPRINTFS(Annotate, tc->getSystemPtr(), "swIdentify: id %#X\n", smi_string);

    add(OP_IDENT, FL_NONE, tc->contextId(), smi, 0, smi_string);
}

uint64_t
CPA::swGetId(ThreadContext *tc)
{
    if (!enabled())
        return 0;

    uint64_t id = ++conId;
    int sysi = getSys(tc->getSystemPtr());
    StackId sid = StackId(sysi, getFrame(tc));
    if (!smStack[sid].size())
        panic("swGetId called without a state machine stack!");
    int smi = smStack[sid].back();

    DPRINTFS(Annotate, tc->getSystemPtr(), "swGetId: id %#X\n", id);

    add(OP_IDENT, FL_NONE, tc->contextId(), smi, 0, id);
    return id;
}


void
CPA::swSyscallLink(ThreadContext  *tc, Addr lsm_string, Addr sm_string)
{
    if (!enabled())
        return;

    char lsm[50];
    tc->getVirtProxy().readString(lsm, lsm_string, 50);
    System *sys = tc->getSystemPtr();
    StringWrap name(sys->name());
    int sysi = getSys(sys);

    Id id = Id(lsm, getFrame(tc));
    StackId sid = StackId(sysi, getFrame(tc));

    if (!smStack[sid].size())
        return;

    int smi = smStack[sid].back();

    DPRINTF(Annotate, "Linking from %d to state machine %s(UNKNOWN)\n",
            smi, lsm);

    if (scLinks[sysi-1][id])
        DPRINTF(Annotate,
                "scLinks already contains entry for system %d %s[%x] of %d\n",
                sysi, lsm, getFrame(tc), scLinks[sysi-1][id]);
    assert(scLinks[sysi-1][id] == 0);
    scLinks[sysi-1][id] = add(OP_LINK, FL_NONE, tc->contextId(), smi, 0xFFFF);
    scLinks[sysi-1][id]->dump = false;

    if (!!sm_string) {
        char sm[50];
        tc->getVirtProxy().readString(sm, sm_string, 50);
        doSwSmEnd(tc->getSystemPtr(), tc->contextId(), sm, getFrame(tc));
    }
}

CPA::AnnDataPtr
CPA::add(int t, int f, int c, int sm, int stq, int32_t d)
{
    AnnDataPtr an = std::make_shared<AnnotateData>();
    an->time = curTick();
    an->data = d;
    an->orig_data = d;
    an->op = t;
    an->flag = f;
    an->sm = sm;
    an->stq = stq;
    an->cpu = c;
    an->dump = true;

    data.push_back(an);

    DPRINTF(AnnotateVerbose, "Annotate: op: %d flags: 0x%x sm: %d state: %d time: %d, data: %d\n",
            an->op, an->flag, an->sm, an->stq, an->time, an->data);

    // Don't dump Links because we might be setting no-dump on it
    if (an->op != OP_LINK)
        dump(false);

    return an;
}

void
CPA::dumpKey()
{
    std::streampos curpos = osbin->tellp();
    ah.key_off = curpos;

    // Output the various state machines and their corresponding states
    *osbin << "# Automatically generated state machine descriptor file" << endl;

    *osbin << "sms = {}" << endl << endl;
    vector<string> state_machines;
    state_machines.resize(numSmt+1);

    // State machines, id -> states
    SCache::iterator i = smtCache.begin();
    while (i != smtCache.end()) {
        state_machines[i->second] = i->first;
        i++;
    }

    for (int x = 1; x < state_machines.size(); x++) {
        vector<string> states;
        states.resize(numSt[x-1]+1);
        assert(x-1 < stCache.size());
        SCache::iterator i = stCache[x-1].begin();
        while (i != stCache[x-1].end()) {
            states[i->second] = i->first;
            i++;
        }
        *osbin << "sms[\"" << state_machines[x] << "\"] = [\"NULL\"";
        for (int y = 1; y < states.size(); y++)
            *osbin << ", \"" << states[y] << "\"";
        *osbin << "]" << endl;
    }

    *osbin << endl << endl << endl;

    // state machine number -> system, name, id
    *osbin << "smNum = [\"NULL\"";
    for (int x = 0; x < smMap.size(); x++)
        *osbin << ", (" << smMap[x].first << ", \"" << smMap[x].second.first <<
            "\", " << smMap[x].second.second << ")";
    *osbin << "]" << endl;

    *osbin << endl << endl << endl;

    // Output the systems
    vector<string> systems;
    systems.resize(numSys+1);
    NameCache::iterator i2 = nameCache.begin();
    while (i2 != nameCache.end()) {
        systems[i2->second.second] = i2->second.first;
        i2++;
    }

    *osbin << "sysNum = [\"NULL\"";
    for (int x = 1; x < systems.size(); x++) {
        *osbin << ", \"" << systems[x] << "\"";
    }
    *osbin << "]" << endl;

    // queue number -> system, qname, qid
    *osbin << "queues = [\"NULL\"";
    for (int x = 0; x < qMap.size(); x++)
        *osbin << ", (" << qMap[x].first << ", \"" << qMap[x].second.first <<
            "\", " << qMap[x].second.second << ")";
    *osbin << "]" << endl;

    *osbin << "smComb = [s for s in [(i,r) for i in xrange(1,len(sysNum)) "
           << "for r in xrange (1,len(smNum))]]" << endl;
    ah.key_len = osbin->tellp() - curpos;

    // output index
    curpos = osbin->tellp();
    ah.idx_off = curpos;

    for (int x = 0; x < annotateIdx.size(); x++)
        osbin->write((char*)&annotateIdx[x], sizeof(uint64_t));
    ah.idx_len = osbin->tellp() - curpos;

    osbin->seekp(0);
    osbin->write((char*)&ah, sizeof(AnnotateHeader));
    osbin->flush();

}

void
CPA::dump(bool all)
{

    list<AnnDataPtr>::iterator i;

    i = data.begin();

    if (i == data.end())
        return;

    // Dump the data every
    if (!all && data.size() < 10000)
        return;

    DPRINTF(Annotate, "Writing %d\n", data.size());
    while (i != data.end()) {
        AnnDataPtr an = *i;

        // If we can't dump this record, hold here
        if (!an->dump && !all)
            break;

        ah.num_recs++;
        if (ah.num_recs % 100000 == 0)
            annotateIdx.push_back(osbin->tellp());


        osbin->write((char*)&(an->time), sizeof(an->time));
        osbin->write((char*)&(an->orig_data), sizeof(an->orig_data));
        osbin->write((char*)&(an->sm), sizeof(an->sm));
        osbin->write((char*)&(an->stq), sizeof(an->stq));
        osbin->write((char*)&(an->op), sizeof(an->op));
        osbin->write((char*)&(an->flag), sizeof(an->flag));
        osbin->write((char*)&(an->cpu), sizeof(an->cpu));
        i++;
    }
    if (data.begin() != i)
        data.erase(data.begin(), i);

    if (all)
        osbin->flush();
}

void
CPA::doQ(System *sys, int flags, int cpuid, int sm,
              string q, int qi, int count)
{
    qSize[qi-1]++;
    qBytes[qi-1] += count;
    if (qSize[qi-1] > 2501 || qBytes[qi-1] > 2000000000)
        warn("Queue %s is %d elements/%d bytes, "
                "maybe things aren't being removed?\n",
                q, qSize[qi-1], qBytes[qi-1]);
    if (flags & FL_QOPP)
        qData[qi-1].push_front(add(OP_QUEUE, flags, cpuid, sm, qi, count));
    else
        qData[qi-1].push_back(add(OP_QUEUE, flags, cpuid, sm, qi, count));
    DPRINTFS(AnnotateQ, sys, "Queing in queue %s size now %d/%d\n",
            q, qSize[qi-1], qBytes[qi-1]);
    assert(qSize[qi-1] >= 0);
    assert(qBytes[qi-1] >= 0);
}


void
CPA::doDq(System *sys, int flags, int cpuid, int sm,
               string q, int qi, int count)
{

    StringWrap name(sys->name());
    if (count == -1) {
        add(OP_DEQUEUE, flags, cpuid, sm, qi, count);
        qData[qi-1].clear();
        qSize[qi-1] = 0;
        qBytes[qi-1] = 0;
        DPRINTF(AnnotateQ, "Dequeing all data in queue %s size now %d/%d\n",
                q, qSize[qi-1], qBytes[qi-1]);
        return;
    }

    assert(count > 0);
    if (qSize[qi-1] <= 0 || qBytes[qi-1] <= 0 || !qData[qi-1].size()) {
        dump(true);
        dumpKey();
        fatal("Queue %s dequing with no data available in queue!\n",
                q);
    }
    assert(qSize[qi-1] >= 0);
    assert(qBytes[qi-1] >= 0);
    assert(qData[qi-1].size());

    int32_t need = count;
    qBytes[qi-1] -= count;
    if (qBytes[qi-1] < 0) {
        dump(true);
        dumpKey();
        fatal("Queue %s dequing with no bytes available in queue!\n",
                q);
    }

    while (need > 0) {
        int32_t head_bytes = qData[qi-1].front()->data;
        if (qSize[qi-1] <= 0 || qBytes[qi-1] < 0) {
            dump(true);
            dumpKey();
            fatal("Queue %s dequing with nothing in queue!\n",
                q);
        }

        if (head_bytes > need) {
            qData[qi-1].front()->data -= need;
            need = 0;
        } else if (head_bytes == need) {
            qData[qi-1].pop_front();
            qSize[qi-1]--;
            need = 0;
        } else {
            qData[qi-1].pop_front();
            qSize[qi-1]--;
            need -= head_bytes;
        }
    }

    add(OP_DEQUEUE, flags, cpuid, sm, qi, count);
    DPRINTF(AnnotateQ, "Dequeing in queue %s size now %d/%d\n",
            q, qSize[qi-1], qBytes[qi-1]);
}



void
CPA::serialize(CheckpointOut &cp) const
{

    SERIALIZE_SCALAR(numSm);
    SERIALIZE_SCALAR(numSmt);
    arrayParamOut(os, "numSt", numSt);
    arrayParamOut(os, "numQ", numQ);
    SERIALIZE_SCALAR(numSys);
    SERIALIZE_SCALAR(numQs);
    SERIALIZE_SCALAR(conId);
    arrayParamOut(os, "qSize", qSize);
    arrayParamOut(os, "qSize", qSize);
    arrayParamOut(os, "qBytes", qBytes);

    SCache::iterator i;
    int x = 0, y = 0;

    // smtCache (SCache)
    x = 0;
    y = 0;
    i = smtCache.begin();
    while (i != smtCache.end()) {
        paramOut(os, csprintf("smtCache%d.str", x), i->first);
        paramOut(os, csprintf("smtCache%d.int", x), i->second);
        x++; i++;
    }

    // stCache  (StCache)
    for (x = 0; x < stCache.size(); x++) {
        i = stCache[x].begin();
        y = 0;
        while (i != stCache[x].end()) {
            paramOut(os, csprintf("stCache%d_%d.str", x, y), i->first);
            paramOut(os, csprintf("stCache%d_%d.int", x, y), i->second);
            y++; i++;
        }
    }

    // qCache (IdCache)
    IdHCache::iterator idi;
    for (x = 0; x < qCache.size(); x++) {
        idi = qCache[x].begin();
        y = 0;
        while (idi != qCache[x].end()) {
            paramOut(os, csprintf("qCache%d_%d.str", x, y), idi->first.first);
            paramOut(os, csprintf("qCache%d_%d.id", x, y), idi->first.second);
            paramOut(os, csprintf("qCache%d_%d.int", x, y), idi->second);
            y++; idi++;
        }
    }

    // smCache (IdCache)
    for (x = 0; x < smCache.size(); x++) {
        idi = smCache[x].begin();
        y = 0;
        paramOut(os, csprintf("smCache%d", x), smCache[x].size());
        while (idi != smCache[x].end()) {
            paramOut(os, csprintf("smCache%d_%d.str", x, y), idi->first.first);
            paramOut(os, csprintf("smCache%d_%d.id", x, y), idi->first.second);
            paramOut(os, csprintf("smCache%d_%d.int", x, y), idi->second);
            y++; idi++;
        }
    }

    // scLinks (ScCache) -- data not serialize


    // namecache (NameCache)
    NameCache::iterator ni;

    ni = nameCache.begin();
    x = 0;
    while (ni != nameCache.end()) {
        paramOut(os, csprintf("nameCache%d.name", x), ni->first->name());
        paramOut(os, csprintf("nameCache%d.str", x), ni->second.first);
        paramOut(os, csprintf("nameCache%d.int", x), ni->second.second);
        x++; ni++;
    }

    // smStack (SmStack)
    SmStack::iterator si;
    si = smStack.begin();
    x = 0;
    paramOut(os, "smStackIdCount", smStack.size());
    while (si != smStack.end()) {
        paramOut(os, csprintf("smStackId%d.sys", x), si->first.first);
        paramOut(os, csprintf("smStackId%d.frame", x), si->first.second);
        paramOut(os, csprintf("smStackId%d.count", x), si->second.size());
        for (y = 0; y < si->second.size(); y++)
            paramOut(os, csprintf("smStackId%d_%d", x, y), si->second[y]);
        x++; si++;
    }

    // lnMap (LinkMap)
    x = 0;
    LinkMap::iterator li;
    li = lnMap.begin();
    paramOut(os, "lnMapSize", lnMap.size());
    while (li != lnMap.end()) {
        paramOut(os, csprintf("lnMap%d.smi", x), li->first);
        paramOut(os, csprintf("lnMap%d.lsmi", x), li->second);
        x++; li++;
    }

    // swExpl (vector)
    SwExpl::iterator swexpli;
    swexpli = swExpl.begin();
    x = 0;
    paramOut(os, "swExplCount", swExpl.size());
    while (swexpli != swExpl.end()) {
        paramOut(os, csprintf("swExpl%d.sys", x), swexpli->first.first);
        paramOut(os, csprintf("swExpl%d.frame", x), swexpli->first.second);
        paramOut(os, csprintf("swExpl%d.swexpl", x), swexpli->second);
        x++; swexpli++;
    }

    // lastState (IMap)
    x = 0;
    IMap::iterator ii;
    ii = lastState.begin();
    paramOut(os, "lastStateSize", lastState.size());
    while (ii != lastState.end()) {
        paramOut(os, csprintf("lastState%d.smi", x), ii->first);
        paramOut(os, csprintf("lastState%d.sti", x), ii->second);
        x++; ii++;
    }

    // smMap (IdMap)
    for (x = 0; x < smMap.size(); x++) {
        paramOut(os, csprintf("smMap%d.sys", x), smMap[x].first);
        paramOut(os, csprintf("smMap%d.smname", x), smMap[x].second.first);
        paramOut(os, csprintf("smMap%d.id", x), smMap[x].second.second);
    }

    // qMap (IdMap)
    for (x = 0; x < qMap.size(); x++) {
        paramOut(os, csprintf("qMap%d.sys", x), qMap[x].first);
        paramOut(os, csprintf("qMap%d.qname", x), qMap[x].second.first);
        paramOut(os, csprintf("qMap%d.id", x), qMap[x].second.second);
    }

    // qData (vector<AnnotateList>)
    for (x = 0; x < qData.size(); x++) {
        if (!qData[x].size())
            continue;
        y = 0;
        for (auto &ann : qData[x]) {
            ann->serializeSection(os, csprintf("Q%d_%d", x, y));
            y++;
        }
    }
}

void
CPA::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(numSm);
    UNSERIALIZE_SCALAR(numSmt);
    UNSERIALIZE_CONTAINER(numSt);
    UNSERIALIZE_CONTAINER(numQ);
    UNSERIALIZE_SCALAR(numSys);
    UNSERIALIZE_SCALAR(numQs);
    UNSERIALIZE_SCALAR(conId);
    UNSERIALIZE_CONTAINER(qSize);
    UNSERIALIZE_CONTAINER(qBytes);


    // smtCache (SCache
    string str;
    int smi;
    for (int x = 0;  x < numSmt; x++) {
        paramIn(cp, csprintf("smtCache%d.str", x), str);
        paramIn(cp, csprintf("smtCache%d.int", x), smi);
        smtCache[str] = smi;
    }

    // stCache  (StCache)
    stCache.resize(numSmt);
    for (int x = 0;  x < numSmt; x++) {
        for (int y = 0; y < numSt[x]; y++) {
            paramIn(cp, csprintf("stCache%d_%d.str", x,y), str);
            paramIn(cp, csprintf("stCache%d_%d.int", x,y), smi);
            stCache[x][str] = smi;
        }
    }

    // qCache (IdCache)
    uint64_t id;
    qCache.resize(numSys);
    for (int x = 0;  x < numSys; x++) {
        for (int y = 0; y < numQ[x]; y++) {
            paramIn(cp, csprintf("qCache%d_%d.str", x,y), str);
            paramIn(cp, csprintf("qCache%d_%d.id", x,y), id);
            paramIn(cp, csprintf("qCache%d_%d.int", x,y), smi);
            qCache[x][Id(str,id)] = smi;
        }
    }

    // smCache (IdCache)
    smCache.resize(numSys);
    for (int x = 0;  x < numSys; x++) {
        int size;
        paramIn(cp, csprintf("smCache%d", x), size);
        for (int y = 0; y < size; y++) {
            paramIn(cp, csprintf("smCache%d_%d.str", x,y), str);
            paramIn(cp, csprintf("smCache%d_%d.id", x,y), id);
            paramIn(cp, csprintf("smCache%d_%d.int", x,y), smi);
            smCache[x][Id(str,id)] = smi;
        }
    }

    // scLinks (ScCache) -- data not serialized, just creating one per sys
    for (int x = 0; x < numSys; x++)
        scLinks.push_back(ScHCache());

    // nameCache (NameCache)
    for (int x = 0; x < numSys; x++) {
        System *sys;
        SimObject *sptr;
        string str;
        int sysi;

        objParamIn(cp, csprintf("nameCache%d.name", x), sptr);
        sys = dynamic_cast<System*>(sptr);

        paramIn(cp, csprintf("nameCache%d.str", x), str);
        paramIn(cp, csprintf("nameCache%d.int", x), sysi);
        nameCache[sys] = std::make_pair(str, sysi);
    }

    //smStack (SmStack)
    int smStack_size;
    paramIn(cp, "smStackIdCount", smStack_size);
    for (int x = 0; x < smStack_size; x++) {
        int sysi;
        uint64_t frame;
        int count;
        paramIn(cp, csprintf("smStackId%d.sys", x), sysi);
        paramIn(cp, csprintf("smStackId%d.frame", x), frame);
        paramIn(cp, csprintf("smStackId%d.count", x), count);
        StackId sid = StackId(sysi, frame);
        for (int y = 0; y < count; y++) {
            paramIn(cp, csprintf("smStackId%d_%d", x, y), smi);
            smStack[sid].push_back(smi);
        }
    }

    // lnMap (LinkMap)
    int lsmi;
    int lnMap_size;
    paramIn(cp, "lnMapSize", lnMap_size);
    for (int x = 0;  x < lnMap_size; x++) {
        paramIn(cp, csprintf("lnMap%d.smi", x), smi);
        paramIn(cp, csprintf("lnMap%d.lsmi", x), lsmi);
        lnMap[smi] = lsmi;
    }

    // swExpl (vector)
    int swExpl_size;
    paramIn(cp, "swExplCount", swExpl_size);
    for (int x = 0; x < swExpl_size; x++) {
        int sysi;
        uint64_t frame;
        bool b;
        paramIn(cp, csprintf("swExpl%d.sys", x), sysi);
        paramIn(cp, csprintf("swExpl%d.frame", x), frame);
        paramIn(cp, csprintf("swExpl%d.swexpl", x), b);
        StackId sid = StackId(sysi, frame);
        swExpl[sid] = b;
    }

    // lastState (IMap)
    int sti;
    int lastState_size;
    paramIn(cp, "lastStateSize", lastState_size);
    for (int x = 0;  x < lastState_size; x++) {
        paramIn(cp, csprintf("lastState%d.smi", x), smi);
        paramIn(cp, csprintf("lastState%d.sti", x), sti);
        lastState[smi] = sti;
    }


    //smMap (IdMap)
    smMap.resize(numSm);
    for (int x = 0; x < smMap.size(); x++) {
        paramIn(cp, csprintf("smMap%d.sys", x), smMap[x].first);
        paramIn(cp, csprintf("smMap%d.smname", x), smMap[x].second.first);
        paramIn(cp, csprintf("smMap%d.id", x), smMap[x].second.second);
    }

    //qMap (IdMap)
    qMap.resize(numQs);
    for (int x = 0; x < qMap.size(); x++) {
        paramIn(cp, csprintf("qMap%d.sys", x), qMap[x].first);
        paramIn(cp, csprintf("qMap%d.qname", x), qMap[x].second.first);
        paramIn(cp, csprintf("qMap%d.id", x), qMap[x].second.second);
    }


    // qData (vector<AnnotateList>)
    qData.resize(qSize.size());
    for (int x = 0; x < qSize.size(); x++) {
        if (!qSize[x])
            continue;
        for (int y = 0; y < qSize[x]; y++) {
            AnnDataPtr a = std::make_shared<AnnotateData>();
            a->unserializeSection(cp, csprintf("Q%d_%d", x, y));
            data.push_back(a);
            qData[x].push_back(a);
        }
    }
}

void
CPA::AnnotateData::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(time);
    SERIALIZE_SCALAR(data);
    SERIALIZE_SCALAR(sm);
    SERIALIZE_SCALAR(stq);
    SERIALIZE_SCALAR(op);
    SERIALIZE_SCALAR(flag);
    SERIALIZE_SCALAR(cpu);
}

void
CPA::AnnotateData::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(time);
    UNSERIALIZE_SCALAR(data);
    orig_data = data;
    UNSERIALIZE_SCALAR(sm);
    UNSERIALIZE_SCALAR(stq);
    UNSERIALIZE_SCALAR(op);
    UNSERIALIZE_SCALAR(flag);
    UNSERIALIZE_SCALAR(cpu);
    dump = true;
}

CPA*
CPAParams::create()
{
    return new CPA(this);
}

