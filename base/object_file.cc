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

#include <list>
#include <string>

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "cprintf.hh"
#include "ecoff.hh"
#include "object_file.hh"
#include "symtab.hh"

using namespace std;

ObjectFile::ObjectFile()
    : descriptor(-1), data(NULL)
{}

ObjectFile::ObjectFile(string file)
    : descriptor(-1), data(NULL)
{ open(file); }

ObjectFile::~ObjectFile()
{ close(); }

bool
ObjectFile::open(string file_name)
{
    close();

    name = file_name;

    descriptor = ::open(name.c_str(), O_RDONLY);
    if (descriptor < 0)
        return false;

    len = (size_t)::lseek(descriptor, 0, SEEK_END);

    data = (uint8_t *)::mmap(NULL, len, PROT_READ, MAP_SHARED, descriptor, 0);
    if (data == MAP_FAILED)
        return false;

    postOpen();

    return true;
}

void
ObjectFile::close()
{
    if (descriptor >= 0)
        ::close(descriptor);

    if (data)
        ::munmap(data, len);
}

void
EcoffObject::postOpen()
{
    exec = &(((EcoffExecHeader *)data)->f);
    aout = &(((EcoffExecHeader *)data)->a);

    text_off = aout->text_start;
    data_off = aout->data_start;
    bss_off = aout->bss_start;

    text_size = aout->tsize;
    data_size = aout->dsize;
    bss_size = aout->bsize;
}

bool
EcoffObject::loadGlobals(SymbolTable *symtab)
{
    if (!symtab)
        return false;

    if (exec->f_magic != ALPHAMAGIC) {
        cprintf("wrong magic\n");
        return false;
    }

    EcoffSymHeader *syms = (EcoffSymHeader *)(data + exec->f_symptr);
    if (syms->magic != ECOFF_SYM_MAGIC) {
        cprintf("bad symbol header magic\n");
        exit(1);
    }

    EcoffExtSymEntry *ext_syms =
        (EcoffExtSymEntry *)(data + syms->cbExtOffset);

    char *ext_strings = (char *)(data + syms->cbSsExtOffset);
    for (int i = 0; i < syms->iextMax; i++) {
        EcoffSymEntry *entry = &(ext_syms[i].asym);
        if (entry->iss != -1)
            symtab->insert(entry->value, ext_strings + entry->iss);
    }

    return true;
}

bool
EcoffObject::loadLocals(SymbolTable *symtab)
{
    if (!symtab)
        return false;

    if (exec->f_magic != ALPHAMAGIC) {
        cprintf("wrong magic\n");
        return false;
    }

    EcoffSymHeader *syms = (EcoffSymHeader *)(data + exec->f_symptr);
    if (syms->magic != ECOFF_SYM_MAGIC) {
        cprintf("bad symbol header magic\n");
        exit(1);
    }

    EcoffSymEntry *local_syms = (EcoffSymEntry *)(data + syms->cbSymOffset);
    char *local_strings = (char *)(data + syms->cbSsOffset);
    EcoffFileDesc *fdesc = (EcoffFileDesc *)(data + syms->cbFdOffset);

    for (int i = 0; i < syms->ifdMax; i++) {
        EcoffSymEntry *entry =
            (EcoffSymEntry *)(local_syms + fdesc[i].isymBase);
        char *strings = (char *)(local_strings + fdesc[i].issBase);
        for (int j = 0; j < fdesc[i].csym; j++) {
            if (entry[j].st == 1 || entry[j].st == 6)
                if (entry[j].iss != -1)
                    symtab->insert(entry[j].value, strings + entry[j].iss);
        }
    }

    for (int i = 0; i < syms->isymMax; i++) {
        EcoffSymEntry *entry = &(local_syms[i]);
        if (entry->st == 6)
            if (entry->st == 1 || entry->st == 6)
                symtab->insert(entry->value, local_strings + entry->iss);
    }

    return true;
}
