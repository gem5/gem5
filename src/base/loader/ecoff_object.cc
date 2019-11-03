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

#include "base/loader/ecoff_object.hh"

#include <string>

#include "base/loader/symtab.hh"
#include "base/logging.hh"
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
EcoffObjectFormat::load(ImageFileDataPtr ifd)
{
    if (((const ecoff_filehdr *)ifd->data())->f_magic == ECOFF_MAGIC_ALPHA)
        return new EcoffObject(ifd);
    else
        return nullptr;
}

namespace
{

EcoffObjectFormat ecoffObjectFormat;

} // anonymous namespace


EcoffObject::EcoffObject(ImageFileDataPtr ifd) : ObjectFile(ifd)
{
    execHdr = (const ecoff_exechdr *)imageData->data();
    fileHdr = &(execHdr->f);
    aoutHdr = &(execHdr->a);

    entry = aoutHdr->entry;
    // it's Alpha ECOFF
    arch = Alpha;
    opSys = Tru64;
}

MemoryImage
EcoffObject::buildImage() const
{
    MemoryImage image({
            { "text", aoutHdr->text_start, imageData,
              ECOFF_TXTOFF(execHdr), aoutHdr->tsize },
            { "data", aoutHdr->data_start, imageData,
              ECOFF_DATOFF(execHdr), aoutHdr->dsize },
            { "bss", aoutHdr->bss_start, aoutHdr->bsize }
    });

    for (auto M5_VAR_USED &seg: image.segments())
        DPRINTFR(Loader, "%s\n", seg);

    return image;
}

bool
EcoffObject::loadAllSymbols(SymbolTable *symtab, Addr base, Addr offset,
                            Addr addr_mask)
{
    bool retval = loadGlobalSymbols(symtab, base, offset, addr_mask);
    retval = retval && loadLocalSymbols(symtab, base, offset, addr_mask);
    return retval;
}

bool
EcoffObject::loadGlobalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                               Addr addr_mask)
{
    if (!symtab)
        return false;

    if (fileHdr->f_magic != ECOFF_MAGIC_ALPHA) {
        warn("loadGlobalSymbols: wrong magic on %s\n", imageData->filename());
        return false;
    }

    auto *syms = (const ecoff_symhdr *)(imageData->data() + fileHdr->f_symptr);
    if (syms->magic != magicSym2) {
        warn("loadGlobalSymbols: bad symbol header magic on %s\n",
                imageData->filename());
        return false;
    }

    auto *ext_syms = (const ecoff_extsym *)(
            imageData->data() + syms->cbExtOffset);

    auto *ext_strings =
        (const char *)(imageData->data() + syms->cbSsExtOffset);
    for (int i = 0; i < syms->iextMax; i++) {
        const ecoff_sym *entry = &(ext_syms[i].asym);
        if (entry->iss != -1)
            symtab->insert(entry->value, ext_strings + entry->iss);
    }

    return true;
}

bool
EcoffObject::loadLocalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                              Addr addr_mask)
{
    if (!symtab)
        return false;

    if (fileHdr->f_magic != ECOFF_MAGIC_ALPHA) {
        warn("loadGlobalSymbols: wrong magic on %s\n", imageData->filename());
        return false;
    }

    auto *syms = (const ecoff_symhdr *)(imageData->data() + fileHdr->f_symptr);
    if (syms->magic != magicSym2) {
        warn("loadGlobalSymbols: bad symbol header magic on %s\n",
                imageData->filename());
        return false;
    }

    auto *local_syms =
        (const ecoff_sym *)(imageData->data() + syms->cbSymOffset);
    auto *local_strings = (const char *)(imageData->data() + syms->cbSsOffset);
    auto *fdesc = (const ecoff_fdr *)(imageData->data() + syms->cbFdOffset);

    for (int i = 0; i < syms->ifdMax; i++) {
        auto *entry = (const ecoff_sym *)(local_syms + fdesc[i].isymBase);
        auto *strings = (const char *)(local_strings + fdesc[i].issBase);
        for (int j = 0; j < fdesc[i].csym; j++) {
            if (entry[j].st == stGlobal || entry[j].st == stProc)
                if (entry[j].iss != -1)
                    symtab->insert(entry[j].value, strings + entry[j].iss);
        }
    }

    for (int i = 0; i < syms->isymMax; i++) {
        const ecoff_sym *entry = &(local_syms[i]);
        if (entry->st == stProc)
            symtab->insert(entry->value, local_strings + entry->iss);
    }

    return true;
}
