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

#include "cpu/exec_context.hh"
#include "targetarch/vtophys.hh"
#include "sim/param.hh"
#include "sim/system.hh"
#include "base/trace.hh"

using namespace std;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(const std::string _name,
               const uint64_t _init_param,
               MemoryController *_memCtrl,
               PhysicalMemory *_physmem,
               const bool _bin,
               const std::vector<string> &binned_fns)

    : SimObject(_name),
      init_param(_init_param),
      memCtrl(_memCtrl),
      physmem(_physmem),
      bin(_bin),
      binned_fns(binned_fns)
{
    // add self to global system list
    systemList.push_back(this);
    if (bin == true) {
        Kernel = new Stats::MainBin("non TCPIP Kernel stats");
        Kernel->activate();
        User = new Stats::MainBin("User stats");

        int end = binned_fns.size();
        assert(!(end & 1));

        Stats::MainBin *Bin;

        fnEvents.resize(end>>1);

        for (int i = 0; i < end; i +=2) {
            Bin = new Stats::MainBin(binned_fns[i]);
            fnBins.insert(make_pair(binned_fns[i], Bin));

            fnEvents[(i>>1)] = new FnEvent(&pcEventQueue, binned_fns[i], this);

            if (binned_fns[i+1] == "null")
                populateMap(binned_fns[i], "");
            else
                populateMap(binned_fns[i], binned_fns[i+1]);
        }

        fnCalls
            .name(name() + ":fnCalls")
            .desc("all fn calls being tracked")
            ;

    } else
        Kernel = NULL;
}


System::~System()
{
    if (bin == true) {
        int end = fnEvents.size();
        for (int i = 0; i < end; ++i) {
            delete fnEvents[i];
        }
        fnEvents.clear();
    }
}


int
System::registerExecContext(ExecContext *xc)
{
    int myIndex = execContexts.size();
    execContexts.push_back(xc);
    return myIndex;
}


void
System::replaceExecContext(int xcIndex, ExecContext *xc)
{
    if (xcIndex >= execContexts.size()) {
        panic("replaceExecContext: bad xcIndex, %d >= %d\n",
              xcIndex, execContexts.size());
    }

    execContexts[xcIndex] = xc;
}


void
System::printSystems()
{
    vector<System *>::iterator i = systemList.begin();
    vector<System *>::iterator end = systemList.end();
    for (; i != end; ++i) {
        System *sys = *i;
        cerr << "System " << sys->name() << ": " << hex << sys << endl;
    }
}

extern "C"
void
printSystems()
{
    System::printSystems();
}

void
System::populateMap(std::string callee, std::string caller)
{
    multimap<const string, string>::const_iterator i;
    i = callerMap.insert(make_pair(callee, caller));
    assert(i != callerMap.end() && "should not fail populating callerMap");
}

bool
System::findCaller(std::string callee, std::string caller) const
{
    typedef multimap<const std::string, std::string>::const_iterator iter;
    pair<iter, iter> range;

    range = callerMap.equal_range(callee);
    for (iter i = range.first; i != range.second; ++i) {
        if ((*i).second == caller)
            return true;
    }
    return false;
}

void
System::dumpState(ExecContext *xc) const
{
    if (xc->swCtx) {
        stack<fnCall *> copy(xc->swCtx->callStack);
        if (copy.empty())
            return;
        DPRINTF(TCPIP, "xc->swCtx, size: %d:\n", copy.size());
        fnCall *top;
        DPRINTF(TCPIP, "||     call : %d\n",xc->swCtx->calls);
        for (top = copy.top(); !copy.empty(); copy.pop() ) {
            top = copy.top();
            DPRINTF(TCPIP, "||  %13s : %s \n", top->name, top->myBin->name());
        }
    }
}

Stats::MainBin *
System::getBin(const std::string &name)
{
    std::map<const std::string, Stats::MainBin *>::const_iterator i;
    i = fnBins.find(name);
    if (i == fnBins.end())
        panic("trying to getBin %s that is not on system map!", name);
    return (*i).second;
}

SWContext *
System::findContext(Addr pcb)
{
  std::map<Addr, SWContext *>::const_iterator iter;
  iter = swCtxMap.find(pcb);
  if (iter != swCtxMap.end()) {
      SWContext *ctx = (*iter).second;
      assert(ctx != NULL && "should never have a null ctx in ctxMap");
      return ctx;
  } else
      return NULL;
}

void
System::serialize(std::ostream &os)
{
    if (bin == true) {
        map<const Addr, SWContext *>::const_iterator iter, end;
        iter = swCtxMap.begin();
        end = swCtxMap.end();

        int numCtxs = swCtxMap.size();
        SERIALIZE_SCALAR(numCtxs);
        SWContext *ctx;
        for (int i = 0; iter != end; ++i, ++iter) {
            paramOut(os, csprintf("Addr[%d]",i), (*iter).first);
            ctx = (*iter).second;
            paramOut(os, csprintf("calls[%d]",i), ctx->calls);

            stack<fnCall *> *stack = &(ctx->callStack);
            fnCall *top;
            int size = stack->size();
            paramOut(os, csprintf("stacksize[%d]",i), size);
            for (int j=0; j<size; ++j) {
                top = stack->top();
                paramOut(os, csprintf("ctx[%d].stackpos[%d]",i,j),
                         top->name);
                delete top;
                stack->pop();
            }
        }
    }
}

void
System::unserialize(Checkpoint *cp, const std::string &section)
{
    if (bin == true) {
        int numCtxs;
        UNSERIALIZE_SCALAR(numCtxs);

        SWContext *ctx;
        Addr addr;
        int size;
        for(int i = 0; i < numCtxs; ++i) {
            ctx = new SWContext;
            paramIn(cp, section, csprintf("Addr[%d]",i), addr);
            paramIn(cp, section, csprintf("calls[%d]",i), ctx->calls);

            paramIn(cp, section, csprintf("stacksize[%d]",i), size);

            vector<fnCall *> calls;
            fnCall *call;
            for (int j = 0; j < size; ++j) {
                call = new fnCall;
                paramIn(cp, section, csprintf("ctx[%d].stackpos[%d]",i,j),
                        call->name);
                call->myBin = getBin(call->name);
                calls.push_back(call);
            }

            for (int j=size-1; j>=0; --j) {
                ctx->callStack.push(calls[j]);
            }

            addContext(addr, ctx);
        }
    }
}

DEFINE_SIM_OBJECT_CLASS_NAME("System", System)

