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

#include <iostream>
#include <string>
#include <vector>

#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "base/str.hh"

using namespace std;

int
main(int argc, char *argv[])
{
    if (argc != 2 && argc != 3)
        panic("usage: %s <filename> <symbol>\n", argv[0]);

    auto *obj = Loader::createObjectFile(argv[1]);
    if (!obj)
        panic("file not found\n");

    if (argc == 2) {
        for (const Loader::Symbol &symbol: obj->symtab())
            cprintf("%#x %s\n", symbol.address, symbol.name);
    } else {
        string symbol = argv[2];
        Addr address;

        if (symbol[0] == '0' && symbol[1] == 'x') {
            Loader::SymbolTable::const_iterator it;
            if (to_number(symbol, address) &&
                (it = obj->symtab().find(address)) != obj->symtab().end()) {
                cprintf("address = %#x, symbol = %s\n", address, it->name);
            } else {
                cprintf("address = %#x was not found\n", address);
            }
        } else {
            auto it = obj->symtab().find(symbol);
            if (it != obj->symtab().end())
                cprintf("symbol = %s address = %#x\n", symbol, it->address);
            else
                cprintf("symbol = %s was not found\n", symbol);
        }
    }

    return 0;
}
