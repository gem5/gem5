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
 */

#include <string>

// Because of the -Wundef flag we have to do this
#define __LIBELF_INTERNAL__     0
// counterintuitive, but the flag below causes libelf to define
// 64-bit elf types that apparently didn't exist in some older
// versions of Linux.  They seem to be there in 2.4.x, so don't
// set this now (it causes things to break on 64-bit platforms).
#define __LIBELF64_LINUX        0
#define __LIBELF_NEED_LINK_H    0
#define __LIBELF_SYMBOL_VERSIONS 0

#include <libelf/libelf.h>
#include <libelf/gelf.h>

#include "base/loader/elf_object.hh"

#include "mem/translating_port.hh"
#include "base/loader/symtab.hh"

#include "base/trace.hh"	// for DPRINTF


using namespace std;

ObjectFile *
ElfObject::tryFile(const string &fname, int fd, size_t len, uint8_t *data)
{
    Elf *elf;
    GElf_Ehdr ehdr;

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    elf = elf_memory((char*)data,len);
    // will only fail if fd is invalid
    assert(elf != NULL);

    // Check that we actually have a elf file
    if (gelf_getehdr(elf, &ehdr) ==0) {
        DPRINTFR(Loader, "Not ELF\n");
        elf_end(elf);
        return NULL;
    }
    else {
        if (ehdr.e_ident[EI_CLASS] == ELFCLASS32)
            panic("32 bit ELF Binary, Not Supported");
        /* @todo this emachine value isn't offical yet.
         *       so we probably shouldn't check it. */
//        if (ehdr.e_machine != EM_ALPHA)
//            panic("Non Alpha Binary, Not Supported");

        elf_end(elf);

        return new ElfObject(fname, fd, len, data,
                             ObjectFile::Alpha, ObjectFile::Linux);
    }
}


ElfObject::ElfObject(const string &_filename, int _fd,
                     size_t _len, uint8_t *_data,
                     Arch _arch, OpSys _opSys)
    : ObjectFile(_filename, _fd, _len, _data, _arch, _opSys)

{
    Elf *elf;
    GElf_Ehdr ehdr;

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    elf = elf_memory((char*)fileData,len);
    // will only fail if fd is invalid
    assert(elf != NULL);

    // Check that we actually have a elf file
    if (gelf_getehdr(elf, &ehdr) ==0) {
        panic("Not ELF, shouldn't be here");
    }

    entry = ehdr.e_entry;

    // initialize segment sizes to 0 in case they're not present
    text.size = data.size = bss.size = 0;

    for (int i = 0; i < ehdr.e_phnum; ++i) {
        GElf_Phdr phdr;
        if (gelf_getphdr(elf, i, &phdr) == 0) {
            panic("gelf_getphdr failed for section %d", i);
        }

        // for now we don't care about non-loadable segments
        if (!(phdr.p_type & PT_LOAD))
            continue;

        // the headers don't explicitly distinguish text from data,
        // but empirically the text segment comes first.
        if (text.size == 0) {  // haven't seen text segment yet
            text.baseAddr = phdr.p_vaddr;
            text.size = phdr.p_filesz;
            // remember where the data is for loadSections()
            fileTextBits = fileData + phdr.p_offset;
            // if there's any padding at the end that's not in the
            // file, call it the bss.  This happens in the "text"
            // segment if there's only one loadable segment (as for
            // kernel images).
            bss.size = phdr.p_memsz - phdr.p_filesz;
            bss.baseAddr = phdr.p_vaddr + phdr.p_filesz;
        }
        else if (data.size == 0) { // have text, this must be data
            data.baseAddr = phdr.p_vaddr;
            data.size = phdr.p_filesz;
            // remember where the data is for loadSections()
            fileDataBits = fileData + phdr.p_offset;
            // if there's any padding at the end that's not in the
            // file, call it the bss.  Warn if this happens for both
            // the text & data segments (should only have one bss).
            if (phdr.p_memsz - phdr.p_filesz > 0 && bss.size != 0) {
                warn("Two implied bss segments in file!\n");
            }
            bss.size = phdr.p_memsz - phdr.p_filesz;
            bss.baseAddr = phdr.p_vaddr + phdr.p_filesz;
        }
    }

    // should have found at least one loadable segment
    assert(text.size != 0);

    DPRINTFR(Loader, "text: 0x%x %d\ndata: 0x%x %d\nbss: 0x%x %d\n",
             text.baseAddr, text.size, data.baseAddr, data.size,
             bss.baseAddr, bss.size);

    elf_end(elf);

    // We will actually read the sections when we need to load them
}


bool
ElfObject::loadSections(TranslatingPort *memPort, bool loadPhys)
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
        memPort->writeBlobFunctional(textAddr, fileTextBits, text.size, true);
    if (data.size != 0)
        memPort->writeBlobFunctional(dataAddr, fileDataBits, data.size, true);

    return true;
}


bool
ElfObject::loadSomeSymbols(SymbolTable *symtab, int binding)
{
    Elf *elf;
    int sec_idx = 1; // there is a 0 but it is nothing, go figure
    Elf_Scn *section;
    GElf_Shdr shdr;
    Elf_Data *data;
    int count, ii;
    bool found = false;
    GElf_Sym sym;

    if (!symtab)
        return false;

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    elf = elf_memory((char*)fileData,len);

    assert(elf != NULL);

    // Get the first section
    section = elf_getscn(elf, sec_idx);

    // While there are no more sections
    while (section != NULL) {
        gelf_getshdr(section, &shdr);

        if (shdr.sh_type == SHT_SYMTAB) {
            found = true;
            data = elf_getdata(section, NULL);
            count = shdr.sh_size / shdr.sh_entsize;
            DPRINTF(Loader, "Found Symbol Table, %d symbols present\n", count);

            // loop through all the symbols, only loading global ones
            for (ii = 0; ii < count; ++ii) {
                gelf_getsym(data, ii, &sym);
                if (GELF_ST_BIND(sym.st_info) == binding) {
                   symtab->insert(sym.st_value,
                                  elf_strptr(elf, shdr.sh_link, sym.st_name));
                }
            }
        }
        ++sec_idx;
        section = elf_getscn(elf, sec_idx);
    }

    elf_end(elf);

    return found;
}

bool
ElfObject::loadGlobalSymbols(SymbolTable *symtab)
{
    return loadSomeSymbols(symtab, STB_GLOBAL);
}

bool
ElfObject::loadLocalSymbols(SymbolTable *symtab)
{
    return loadSomeSymbols(symtab, STB_LOCAL);
}
