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

#include <string>

#include "base/loader/elf_object.hh"

#include "mem/functional_mem/functional_memory.hh"
#include "base/loader/symtab.hh"

#include "base/trace.hh"	// for DPRINTF

#include "base/loader/exec_elf.h"

using namespace std;

ObjectFile *
ElfObject::tryFile(const string &fname, int fd, size_t len, uint8_t *data)
{
    if (memcmp(((Elf64_Ehdr *)data)->e_ident, ELFMAG, SELFMAG) == 0) {
        // for now we'll assume it's a 64-bit Alpha binary
        return new ElfObject(fname, fd, len, data);
    }
    else {
        return NULL;
    }
}


ElfObject::ElfObject(const string &_filename, int _fd,
                         size_t _len, uint8_t *_data)
    : ObjectFile(_filename, _fd, _len, _data)
{
    ehdr = (Elf64_Ehdr *)fileData;

    entry = ehdr->e_entry;

    phdr = (Elf64_Phdr *)(fileData + ehdr->e_phoff);
    assert(sizeof(Elf64_Phdr) == ehdr->e_phentsize);

    bool foundText = false;
    bool foundData = false;
    for (int i = 0; i < ehdr->e_phnum; ++i) {
        Elf64_Phdr *p = &phdr[i];

        // for now we don't care about non-loadable segments
        if (!(p->p_type & PT_LOAD))
            continue;

        if (p->p_flags & PF_X) {
            // executable: must be text
            assert(!foundText);
            foundText = true;
            textPhdrIdx = i;
            text.baseAddr = p->p_vaddr;
            text.size = p->p_filesz;
            assert(p->p_filesz == p->p_memsz);
        }
        else {
            assert(p->p_flags & PF_R);
            assert(!foundData);
            foundData = true;
            dataPhdrIdx = i;
            data.baseAddr = p->p_vaddr;
            data.size = p->p_filesz;
            bss.baseAddr = data.baseAddr + data.size;
            bss.size = p->p_memsz - p->p_filesz;
        }
    }

    assert(foundText && foundData);

    DPRINTFR(Loader, "text: 0x%x %d\ndata: 0x%x %d\nbss: 0x%x %d\n",
             text.baseAddr, text.size, data.baseAddr, data.size,
             bss.baseAddr, bss.size);
}


bool
ElfObject::loadSections(FunctionalMemory *mem, bool loadPhys)
{
    Addr textAddr = text.baseAddr;
    Addr dataAddr = data.baseAddr;

    if (loadPhys) {
        textAddr &= (ULL(1) << 40) - 1;
        dataAddr &= (ULL(1) << 40) - 1;
    }

    // Since we don't really have an MMU and all memory is
    // zero-filled, there's no need to set up the BSS segment.
    if (text.size != 0)
        mem->prot_write(textAddr, fileData + phdr[textPhdrIdx].p_offset,
                        text.size);
    if (data.size != 0)
        mem->prot_write(dataAddr, fileData + phdr[dataPhdrIdx].p_offset,
                        data.size);

    return true;
}


bool
ElfObject::loadGlobalSymbols(SymbolTable *symtab)
{
    // symbols not supported yet
    return false;
}

bool
ElfObject::loadLocalSymbols(SymbolTable *symtab)
{
    // symbols not supported yet
    return false;
}
