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

#include "libelf/libelf.h"
#include "libelf/gelf.h"

#include "base/loader/elf_object.hh"
#include "base/misc.hh"

#include "base/loader/symtab.hh"

#include "base/trace.hh"	// for DPRINTF

#include "sim/byteswap.hh"


using namespace std;

ObjectFile *
ElfObject::tryFile(const string &fname, int fd, size_t len, uint8_t *data)
{
    Elf *elf;
    GElf_Ehdr ehdr;
    Arch arch = UnknownArch;
    OpSys opSys = UnknownOpSys;

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
    } else {
        //Detect the architecture
        //Since we don't know how to check for alpha right now, we'll
        //just assume if it wasn't something else and it's 64 bit, that's
        //what it must be.
        if (ehdr.e_machine == EM_SPARC64 ||
                ehdr.e_machine == EM_SPARC ||
                ehdr.e_machine == EM_SPARCV9 ||
                ehdr.e_machine == EM_SPARC32PLUS) {
            arch = ObjectFile::SPARC;
        } else if (ehdr.e_machine == EM_MIPS
                && ehdr.e_ident[EI_CLASS] == ELFCLASS32) {
            arch = ObjectFile::Mips;
        } else if (ehdr.e_ident[EI_CLASS] == ELFCLASS64) {
            arch = ObjectFile::Alpha;
        } else {
            arch = ObjectFile::UnknownArch;
        }

        //Detect the operating system
        switch (ehdr.e_ident[EI_OSABI])
        {

          case ELFOSABI_LINUX:
            opSys = ObjectFile::Linux;
            break;
          case ELFOSABI_SOLARIS:
            opSys = ObjectFile::Solaris;
            break;
          case ELFOSABI_TRU64:
            opSys = ObjectFile::Tru64;
            break;
          default:
            opSys = ObjectFile::UnknownOpSys;
        }

        //take a look at the .note.ABI section
        //It can let us know what's what.
        if (opSys == ObjectFile::UnknownOpSys)
        {
            Elf_Scn *section;
            GElf_Shdr shdr;
            Elf_Data *data;
            uint32_t osAbi;;
            int secIdx = 1;

            // Get the first section
            section = elf_getscn(elf, secIdx);

            // While there are no more sections
            while (section != NULL) {
                gelf_getshdr(section, &shdr);
                if (shdr.sh_type == SHT_NOTE && !strcmp(".note.ABI-tag",
                            elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name))) {
                    // we have found a ABI note section
                    // Check the 5th 32bit word for OS  0 == linux, 1 == hurd,
                    // 2 == solaris, 3 == freebsd
                    data = elf_rawdata(section, NULL);
                    assert(data->d_buf);
                    if(ehdr.e_ident[EI_DATA] == ELFDATA2LSB)
                        osAbi = htole(((uint32_t*)data->d_buf)[4]);
                    else
                        osAbi = htobe(((uint32_t*)data->d_buf)[4]);

                    switch(osAbi) {
                      case 0:
                        opSys = ObjectFile::Linux;
                        break;
                      case 2:
                        opSys = ObjectFile::Solaris;
                        break;
                    }
                } // if section found
            section = elf_getscn(elf, ++secIdx);
            } // while sections
        }

        elf_end(elf);
        return new ElfObject(fname, fd, len, data, arch, opSys);
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
            text.fileImage = fileData + phdr.p_offset;
            // if there's any padding at the end that's not in the
            // file, call it the bss.  This happens in the "text"
            // segment if there's only one loadable segment (as for
            // kernel images).
            bss.size = phdr.p_memsz - phdr.p_filesz;
            bss.baseAddr = phdr.p_vaddr + phdr.p_filesz;
            bss.fileImage = NULL;
        } else if (data.size == 0) { // have text, this must be data
            data.baseAddr = phdr.p_vaddr;
            data.size = phdr.p_filesz;
            data.fileImage = fileData + phdr.p_offset;
            // if there's any padding at the end that's not in the
            // file, call it the bss.  Warn if this happens for both
            // the text & data segments (should only have one bss).
            if (phdr.p_memsz - phdr.p_filesz > 0 && bss.size != 0) {
                warn("Two implied bss segments in file!\n");
            }
            bss.size = phdr.p_memsz - phdr.p_filesz;
            bss.baseAddr = phdr.p_vaddr + phdr.p_filesz;
            bss.fileImage = NULL;
        } else {
            warn("More than two loadable segments in ELF object.");
            warn("Ignoring segment @ 0x%x length 0x%x.",
                 phdr.p_vaddr, phdr.p_filesz);
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
