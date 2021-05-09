/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#include "base/loader/symtab.hh"

#include <fstream>
#include <iostream>

#include "base/logging.hh"
#include "base/str.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Loader, loader);
namespace loader
{

SymbolTable debugSymbolTable;

void
SymbolTable::clear()
{
    addrMap.clear();
    nameMap.clear();
    symbols.clear();
}

bool
SymbolTable::insert(const Symbol &symbol)
{
    if (symbol.name.empty())
        return false;

    int idx = symbols.size();

    if (!nameMap.insert({ symbol.name, idx }).second)
        return false;

    // There can be multiple symbols for the same address, so always
    // update the addrTable multimap when we see a new symbol name.
    addrMap.insert({ symbol.address, idx });

    symbols.emplace_back(symbol);

    return true;
}

bool
SymbolTable::insert(const SymbolTable &other)
{
    // Check if any symbol in other already exists in our table.
    NameMap intersection;
    std::set_intersection(other.nameMap.begin(), other.nameMap.end(),
                          nameMap.begin(), nameMap.end(),
                          std::inserter(intersection, intersection.begin()),
                          nameMap.value_comp());
    if (!intersection.empty())
        return false;

    for (const Symbol &symbol: other)
        insert(symbol);

    return true;
}

void
SymbolTable::serialize(const std::string &base, CheckpointOut &cp) const
{
    paramOut(cp, base + ".size", symbols.size());

    int i = 0;
    for (auto &symbol: symbols) {
        paramOut(cp, csprintf("%s.addr_%d", base, i), symbol.address);
        paramOut(cp, csprintf("%s.symbol_%d", base, i), symbol.name);
        paramOut(cp, csprintf("%s.binding_%d", base, i), (int)symbol.binding);
        i++;
    }
}

void
SymbolTable::unserialize(const std::string &base, CheckpointIn &cp,
                         Symbol::Binding default_binding)
{
    clear();
    int size;
    paramIn(cp, base + ".size", size);
    for (int i = 0; i < size; ++i) {
        Addr address;
        std::string name;
        Symbol::Binding binding = default_binding;

        paramIn(cp, csprintf("%s.addr_%d", base, i), address);
        paramIn(cp, csprintf("%s.symbol_%d", base, i), name);
        if (!optParamIn(cp, csprintf("%s.binding_%d", base, i), binding))
            binding = default_binding;
        insert({binding, name, address});
    }
}

} // namespace loader
} // namespace gem5
