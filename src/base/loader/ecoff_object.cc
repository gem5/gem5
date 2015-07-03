/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include <string>

#include "base/loader/ecoff_object.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Loader.hh"

// Only alpha will be able to load ecoff files for now.
// base/types.hh and ecoff_machdep.h must be before the other .h files
// because they are are gathered from other code bases and require some 
// typedefs from those files.
#include "arch/alpha/ecoff_machdep.h"
#include "base/loader/coff_sym.h"
#include "base/loader/coff_symconst.h"
#include "base/loader/exec_ecoff.h"

using namespace std;

ObjectFile *
EcoffObject::tryFile(const string &fname, size_t len, uint8_t *data)
{
    if (((ecoff_filehdr *)data)->f_magic == ECOFF_MAGIC_ALPHA) {
        // it's Alpha ECOFF
        return new EcoffObject(fname, len, data,
                               ObjectFile::Alpha, ObjectFile::Tru64);
    }
    else {
        return NULL;
    }
}


EcoffObject::EcoffObject(const string &_filename, size_t _len, uint8_t *_data,
                         Arch _arch, OpSys _opSys)
    : ObjectFile(_filename, _len, _data, _arch, _opSys)
{
    execHdr = (ecoff_exechdr *)fileData;
    fileHdr = &(execHdr->f);
    aoutHdr = &(execHdr->a);

    entry = aoutHdr->entry;

    text.baseAddr = aoutHdr->text_start;
    text.size = aoutHdr->tsize;
    text.fileImage = fileData + ECOFF_TXTOFF(execHdr);

    data.baseAddr = aoutHdr->data_start;
    data.size = aoutHdr->dsize;
    data.fileImage = fileData + ECOFF_DATOFF(execHdr);

    bss.baseAddr = aoutHdr->bss_start;
    bss.size = aoutHdr->bsize;
    bss.fileImage = NULL;

    DPRINTFR(Loader, "text: 0x%x %d\ndata: 0x%x %d\nbss: 0x%x %d\n",
             text.baseAddr, text.size, data.baseAddr, data.size,
             bss.baseAddr, bss.size);
}


bool
EcoffObject::loadGlobalSymbols(SymbolTable *symtab, Addr addrMask)
{
    if (!symtab)
        return false;

    if (fileHdr->f_magic != ECOFF_MAGIC_ALPHA) {
        warn("loadGlobalSymbols: wrong magic on %s\n", filename);
        return false;
    }

    ecoff_symhdr *syms = (ecoff_symhdr *)(fileData + fileHdr->f_symptr);
    if (syms->magic != magicSym2) {
        warn("loadGlobalSymbols: bad symbol header magic on %s\n", filename);
        return false;
    }

    ecoff_extsym *ext_syms = (ecoff_extsym *)(fileData + syms->cbExtOffset);

    char *ext_strings = (char *)(fileData + syms->cbSsExtOffset);
    for (int i = 0; i < syms->iextMax; i++) {
        ecoff_sym *entry = &(ext_syms[i].asym);
        if (entry->iss != -1)
            symtab->insert(entry->value, ext_strings + entry->iss);
    }

    return true;
}

bool
EcoffObject::loadLocalSymbols(SymbolTable *symtab, Addr addrMask)
{
    if (!symtab)
        return false;

    if (fileHdr->f_magic != ECOFF_MAGIC_ALPHA) {
        warn("loadGlobalSymbols: wrong magic on %s\n", filename);
        return false;
    }

    ecoff_symhdr *syms = (ecoff_symhdr *)(fileData + fileHdr->f_symptr);
    if (syms->magic != magicSym2) {
        warn("loadGlobalSymbols: bad symbol header magic on %s\n", filename);
        return false;
    }

    ecoff_sym *local_syms = (ecoff_sym *)(fileData + syms->cbSymOffset);
    char *local_strings = (char *)(fileData + syms->cbSsOffset);
    ecoff_fdr *fdesc = (ecoff_fdr *)(fileData + syms->cbFdOffset);

    for (int i = 0; i < syms->ifdMax; i++) {
        ecoff_sym *entry = (ecoff_sym *)(local_syms + fdesc[i].isymBase);
        char *strings = (char *)(local_strings + fdesc[i].issBase);
        for (int j = 0; j < fdesc[i].csym; j++) {
            if (entry[j].st == stGlobal || entry[j].st == stProc)
                if (entry[j].iss != -1)
                    symtab->insert(entry[j].value, strings + entry[j].iss);
        }
    }

    for (int i = 0; i < syms->isymMax; i++) {
        ecoff_sym *entry = &(local_syms[i]);
        if (entry->st == stProc)
            symtab->insert(entry->value, local_strings + entry->iss);
    }

    return true;
}
