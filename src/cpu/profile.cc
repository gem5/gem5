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
 */

#include "cpu/profile.hh"

#include <string>

#include "base/callback.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"

namespace gem5
{

void
BaseStackTrace::dump()
{
    StringWrap name(tc->getCpuPtr()->name());
    auto *symtab = &tc->getSystemPtr()->workload->symtab(tc);

    DPRINTFN("------ Stack ------\n");

    std::string symbol;
    for (int i = 0, size = stack.size(); i < size; ++i) {
        Addr addr = stack[size - i - 1];
        getSymbol(symbol, addr, symtab);
        DPRINTFN("%#x: %s\n", addr, symbol);
    }
}

bool
BaseStackTrace::tryGetSymbol(std::string &symbol, Addr addr,
                             const loader::SymbolTable *symtab)
{
    const auto it = symtab->find(addr);
    if (it == symtab->end())
        return false;
    symbol = it->name();
    return true;
}

void
ProfileNode::dump(const std::string &symbol, uint64_t id,
                  const FunctionProfile &prof, std::ostream &os) const
{
    ccprintf(os, "%#x %s %d ", id, symbol, count);
    for (const auto &p : children)
        ccprintf(os, "%#x ", (intptr_t)(p.second));

    ccprintf(os, "\n");

    for (const auto &p : children) {
        Addr addr = p.first;
        std::string symbol;

        prof.trace->getSymbol(symbol, addr, &prof.symtab);

        const auto *node = p.second;
        node->dump(symbol, (intptr_t)node, prof, os);
    }
}

void
ProfileNode::clear()
{
    count = 0;
    for (const auto &p : children)
        p.second->clear();
}

FunctionProfile::FunctionProfile(std::unique_ptr<BaseStackTrace> _trace,
                                 const loader::SymbolTable &_symtab)
    : symtab(_symtab), trace(std::move(_trace))
{
    statistics::registerResetCallback([this]() { clear(); });
}

ProfileNode *
FunctionProfile::consume(const std::vector<Addr> &stack)
{
    ProfileNode *current = &top;
    for (int i = 0, size = stack.size(); i < size; ++i) {
        ProfileNode *&ptr = current->children[stack[size - i - 1]];
        if (!ptr)
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
FunctionProfile::dump(std::ostream &os) const
{
    ccprintf(os, ">>>PC data\n");
    for (const auto &p : pc_count) {
        Addr pc = p.first;
        Counter count = p.second;

        std::string symbol;
        if (trace->tryGetSymbol(symbol, pc, &symtab))
            ccprintf(os, "%s %d\n", symbol, count);
        else
            ccprintf(os, "%#x %d\n", pc, count);
    }

    ccprintf(os, ">>>function data\n");
    top.dump("top", 0, *this, os);
}

void
FunctionProfile::sample(ProfileNode *node, Addr pc)
{
    node->count++;

    auto it = symtab.findNearest(pc);
    if (it != symtab.end()) {
        pc_count[it->address()]++;
    } else {
        // record PC even if we don't have a symbol to avoid
        // silently biasing the histogram
        pc_count[pc]++;
    }
}

} // namespace gem5
