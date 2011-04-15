/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#include <string>

#include "base/loader/symtab.hh"
#include "base/bitfield.hh"
#include "base/callback.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/profile.hh"
#include "cpu/thread_context.hh"

using namespace std;

ProfileNode::ProfileNode()
    : count(0)
{ }

void
ProfileNode::dump(const string &symbol, uint64_t id, const SymbolTable *symtab,
                  ostream &os) const
{
    ccprintf(os, "%#x %s %d ", id, symbol, count);
    ChildList::const_iterator i, end = children.end();
    for (i = children.begin(); i != end; ++i) {
        const ProfileNode *node = i->second;
        ccprintf(os, "%#x ", (intptr_t)node);
    }

    ccprintf(os, "\n");

    for (i = children.begin(); i != end; ++i) {
        Addr addr = i->first;
        string symbol;
        if (addr == 1)
            symbol = "user";
        else if (addr == 2)
            symbol = "console";
        else if (addr == 3)
            symbol = "unknown";
        else if (!symtab->findSymbol(addr, symbol))
            panic("could not find symbol for address %#x\n", addr);

        const ProfileNode *node = i->second;
        node->dump(symbol, (intptr_t)node, symtab, os);
    }
}

void
ProfileNode::clear()
{
    count = 0;
    ChildList::iterator i, end = children.end();
    for (i = children.begin(); i != end; ++i)
        i->second->clear();
}

FunctionProfile::FunctionProfile(const SymbolTable *_symtab)
    : reset(0), symtab(_symtab)
{
    reset = new MakeCallback<FunctionProfile, &FunctionProfile::clear>(this);
    Stats::registerResetCallback(reset);
}

FunctionProfile::~FunctionProfile()
{
    if (reset)
        delete reset;
}

ProfileNode *
FunctionProfile::consume(const vector<Addr> &stack)
{
    ProfileNode *current = &top;
    for (int i = 0, size = stack.size(); i < size; ++i) {
        ProfileNode *&ptr = current->children[stack[size - i - 1]];
        if (ptr == NULL)
            ptr = new ProfileNode;

        current = ptr;
    }

    return current;
}

void
FunctionProfile::clear()
{
    top.clear();
    pc_count.clear();
}

void
FunctionProfile::dump(ThreadContext *tc, ostream &os) const
{
    ccprintf(os, ">>>PC data\n");
    map<Addr, Counter>::const_iterator i, end = pc_count.end();
    for (i = pc_count.begin(); i != end; ++i) {
        Addr pc = i->first;
        Counter count = i->second;

        std::string symbol;
        if (pc == 1)
            ccprintf(os, "user %d\n", count);
        else if (symtab->findSymbol(pc, symbol) && !symbol.empty())
            ccprintf(os, "%s %d\n", symbol, count);
        else
            ccprintf(os, "%#x %d\n", pc, count);
    }

    ccprintf(os, ">>>function data\n");
    top.dump("top", 0, symtab, os);
}

void
FunctionProfile::sample(ProfileNode *node, Addr pc)
{
    node->count++;

    Addr symaddr;
    if (symtab->findNearestAddr(pc, symaddr)) {
        pc_count[symaddr]++;
    } else {
        // record PC even if we don't have a symbol to avoid
        // silently biasing the histogram
        pc_count[pc]++;
    }
}
