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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "sim/host.hh"
#include "base/misc.hh"
#include "base/str.hh"
#include "base/loader/symtab.hh"

using namespace std;

bool
SymbolTable::insert(Addr address, string symbol)
{
    if (!addrTable.insert(make_pair(address, symbol)).second)
        return false;

    if (!symbolTable.insert(make_pair(symbol, address)).second)
        return false;

    return true;
}


bool
SymbolTable::load(const string &filename)
{
    string buffer;
    ifstream file(filename.c_str());

    if (!file) {
        cerr << "Can't open symbol table file " << filename << endl;
        fatal("file error");
    }

    while (!file.eof()) {
        getline(file, buffer);
        if (buffer.empty())
            continue;

        int idx = buffer.find(',');
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

bool
SymbolTable::findNearestSymbol(Addr address, string &symbol) const
{
    ATable::const_iterator i = addrTable.lower_bound(address);

    // check for PALCode
    if (address & 0x1)
        return false;

    // first check for the end
    if (i == addrTable.end())
        i--;
    else if (i == addrTable.begin() && (*i).first != address)
        return false;
    else if ((*i).first != address)
        i--;

    symbol = (*i).second;

    if (address != (*i).first)
        symbol += csprintf("+%d", address - (*i).first);

    return true;
}

bool
SymbolTable::findSymbol(Addr address, string &symbol) const
{
    ATable::const_iterator i = addrTable.find(address);
    if (i == addrTable.end())
        return false;

    symbol = (*i).second;
    return true;
}

bool
SymbolTable::findAddress(const string &symbol, Addr &address) const
{
    STable::const_iterator i = symbolTable.find(symbol);
    if (i == symbolTable.end())
        return false;

    address = (*i).second;
    return true;
}

string
SymbolTable::find(Addr addr) const
{
    string s;
    findSymbol(addr, s);
    return s;
}

Addr
SymbolTable::find(const string &symbol) const
{
    Addr a = 0;
    findAddress(symbol, a);
    return a;
}
