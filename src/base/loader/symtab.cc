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
 *
 * Authors: Nathan Binkert
 */

#include "base/loader/symtab.hh"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/str.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

using namespace std;

SymbolTable *debugSymbolTable = NULL;

void
SymbolTable::clear()
{
    addrTable.clear();
    symbolTable.clear();
}

bool
SymbolTable::insert(Addr address, string symbol)
{
    if (symbol.empty())
        return false;

    if (!symbolTable.insert(make_pair(symbol, address)).second)
        return false;

    // There can be multiple symbols for the same address, so always
    // update the addrTable multimap when we see a new symbol name.
    addrTable.insert(make_pair(address, symbol));

    return true;
}


bool
SymbolTable::load(const string &filename)
{
    string buffer;
    ifstream file(filename.c_str());

    if (!file)
        fatal("file error: Can't open symbol table file %s\n", filename);

    while (!file.eof()) {
        getline(file, buffer);
        if (buffer.empty())
            continue;

        string::size_type idx = buffer.find(',');
        if (idx == string::npos)
            return false;

        string address = buffer.substr(0, idx);
        eat_white(address);
        if (address.empty())
            return false;

        string symbol = buffer.substr(idx + 1);
        eat_white(symbol);
        if (symbol.empty())
            return false;

        Addr addr;
        if (!to_number(address, addr))
            return false;

        if (!insert(addr, symbol))
            return false;
    }

    file.close();

    return true;
}

void
SymbolTable::serialize(const string &base, CheckpointOut &cp) const
{
    paramOut(cp, base + ".size", addrTable.size());

    int i = 0;
    ATable::const_iterator p, end = addrTable.end();
    for (p = addrTable.begin(); p != end; ++p) {
        paramOut(cp, csprintf("%s.addr_%d", base, i), p->first);
        paramOut(cp, csprintf("%s.symbol_%d", base, i), p->second);
        ++i;
    }
}

void
SymbolTable::unserialize(const string &base, CheckpointIn &cp)
{
    clear();
    int size;
    paramIn(cp, base + ".size", size);
    for (int i = 0; i < size; ++i) {
        Addr addr;
        std::string symbol;

        paramIn(cp, csprintf("%s.addr_%d", base, i), addr);
        paramIn(cp, csprintf("%s.symbol_%d", base, i), symbol);
        insert(addr, symbol);
    }
}
