/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

#include "base/loader/raw_object.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "debug/Loader.hh"

ObjectFile *
RawObject::tryFile(const std::string &fname, size_t len, uint8_t *data)
{
    return new RawObject(fname, len, data, ObjectFile::UnknownArch,
            ObjectFile::UnknownOpSys);
}

RawObject::RawObject(const std::string &_filename, size_t _len,
        uint8_t *_data, Arch _arch, OpSys _opSys)
    : ObjectFile(_filename, _len, _data, _arch, _opSys)
{
    text.baseAddr = 0;
    text.size = len;
    text.fileImage = fileData;

    data.baseAddr = 0;
    data.size = 0;
    data.fileImage = NULL;

    bss.baseAddr = 0;
    bss.size = 0;
    bss.fileImage = NULL;

    DPRINTFR(Loader, "text: 0x%x %d\ndata: 0x%x %d\nbss: 0x%x %d\n",
             text.baseAddr, text.size, data.baseAddr, data.size,
             bss.baseAddr, bss.size);
}

bool
RawObject::loadAllSymbols(SymbolTable *symtab, Addr base, Addr offset,
                          Addr addr_mask)
{
    return true;
}

bool
RawObject::loadGlobalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                             Addr addr_mask)
{
/*    int fnameStart = filename.rfind('/',filename.size()) + 1;
    int extStart = filename.rfind('.',filename.size());
    symtab->insert(text.baseAddr & addr_mask, filename.substr(fnameStart,
                extStart-fnameStart) + "_start");*/
    return true;
}

bool
RawObject::loadLocalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                            Addr addr_mask)
{
/*    int fnameStart = filename.rfind('/',filename.size()) + 1;
    int extStart = filename.rfind('.',filename.size());
    symtab->insert(text.baseAddr & addr_mask, filename.substr(fnameStart,
                extStart-fnameStart) + "_start");*/
    return true;
}
