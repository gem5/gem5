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

#include <iostream>
#include <string>
#include <vector>

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/str.hh"

using namespace std;

int
main(int argc, char *argv[])
{
    if (argc != 2 && argc != 3)
        panic("usage: %s <filename> <symbol>\n", argv[0]);

    ObjectFile *obj = createObjectFile(argv[1]);
    if (!obj)
        panic("file not found\n");

    SymbolTable symtab;
    obj->loadGlobalSymbols(&symtab);
    obj->loadLocalSymbols(&symtab);

    if (argc == 2) {
        SymbolTable::ATable::const_iterator i = symtab.getAddrTable().begin();
        SymbolTable::ATable::const_iterator end = symtab.getAddrTable().end();
        while (i != end) {
            cprintf("%#x %s\n", i->first, i->second);
            ++i;
        }
    } else {
        string symbol = argv[2];
        Addr address;

        if (symbol[0] == '0' && symbol[1] == 'x') {
            if (to_number(symbol, address) &&
                symtab.findSymbol(address, symbol))
                cprintf("address = %#x, symbol = %s\n", address, symbol);
            else
                cprintf("address = %#x was not found\n", address);
        } else {
            if (symtab.findAddress(symbol, address))
                cprintf("symbol = %s address = %#x\n", symbol, address);
            else
                cprintf("symbol = %s was not found\n", symbol);
        }
    }

    return 0;
}
