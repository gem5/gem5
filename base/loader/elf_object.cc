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


using namespace std;

ObjectFile *
ElfObject::tryFile(const string &fname, int fd, size_t len, uint8_t *data)
{
    Elf *elf;
    GElf_Ehdr ehdr;


    /* check that header matches library version */
    assert(elf_version(EV_CURRENT) != EV_NONE);

    /* get a pointer to elf structure */
    elf = elf_memory((char*)data,len);
    /* will only fail if fd is invalid */
    assert(elf != NULL);

    /*  Check that we actually have a elf file */
    if(gelf_getehdr(elf, &ehdr) ==0)
    {
        DPRINTFR(Loader, "Not ELF\n");
        elf_end(elf);
        return NULL;
    }
    else
    {
        if (ehdr.e_ident[EI_CLASS] == ELFCLASS32)
            panic("32 bit ELF Binary, Not Supported");
        if (ehdr.e_machine != EM_ALPHA)
            panic("Non Alpha Binary, Not Supported");

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

    /* check that header matches library version */
    assert(elf_version(EV_CURRENT) != EV_NONE);

    /* get a pointer to elf structure */
    elf = elf_memory((char*)fileData,len);
    /* will only fail if fd is invalid */
    assert(elf != NULL);

    /*  Check that we actually have a elf file */
    if(gelf_getehdr(elf, &ehdr) ==0)
    {
        panic("Not ELF, shouldn't be here");
    }


    entry = ehdr.e_entry;
    elf_end(elf);

    /* We will actually read the sections when we need to load them*/
}


bool
ElfObject::loadSections(FunctionalMemory *mem, bool loadPhys)
{
    Elf *elf;
    int secidx = 1; /* there is a 0 but it is nothing, go figure*/
    Elf_Scn *section;
    GElf_Shdr shdr;
    GElf_Ehdr ehdr;

    Addr address;
    char *secname;

    /* check that header matches library version */
    assert(elf_version(EV_CURRENT) != EV_NONE);


    /* get a pointer to elf structure */
    elf = elf_memory((char*)fileData,len);

    assert(elf != NULL);

    /*  Check that we actually have a elf file */
    if(gelf_getehdr(elf, &ehdr) ==0)
    {
        panic("Not ELF, shouldn't be here");
    }



    /* Get the first section */
    section = elf_getscn(elf, secidx);

    /* While there are no more sections */
    while (section != NULL)
    {
        gelf_getshdr(section, &shdr);


        if (shdr.sh_flags & SHF_ALLOC)
        {
            /* we should load this */
            DPRINTF(Loader,"Name: %20s Address: 0x%016llx Size: 0x%08llx Offset: 0x%08llx   Flags:0x%08llx %s\n",
                    elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name), shdr.sh_addr,
                    shdr.sh_size, shdr.sh_offset, shdr.sh_flags, shdr.sh_flags &    SHF_ALLOC ? "ALLOC" : "");
            secname = elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name);
            if(secname)
            {
                if (strcmp(secname, ".text")==0)
                {
                    text.baseAddr = shdr.sh_addr;
                    text.size = shdr.sh_size;
                }
                if (strcmp(secname, ".data")==0)
                {
                    data.baseAddr = shdr.sh_addr;
                    data.size = shdr.sh_size;
                }
                if (strcmp(secname, ".bss")==0)
                {
                    bss.baseAddr = shdr.sh_addr;
                    bss.size = shdr.sh_size;
                }
            }
            if(shdr.sh_size != 0)
            {
                if (loadPhys)
                {
                    address = shdr.sh_addr &= (ULL(1) << 40) - 1;
                    mem->prot_write(address, fileData + shdr.sh_offset, shdr.sh_size);
                }
                else
                {
                    mem->prot_write(shdr.sh_addr, fileData + shdr.sh_offset, shdr.sh_size);
                }
            }

        }

        ++secidx;
        section = elf_getscn(elf, secidx);
    }

    elf_end(elf);

    return true;
}


bool
ElfObject::loadGlobalSymbols(SymbolTable *symtab)
{
    Elf *elf;
    int secidx = 1; /* there is a 0 but it is nothing, go figure*/
    Elf_Scn *section;
    GElf_Shdr shdr;
    Elf_Data *data;
    int count, ii;
    bool found = false;
    GElf_Sym sym;

    if (!symtab)
        return false;

    /* check that header matches library version */
    assert(elf_version(EV_CURRENT) != EV_NONE);

    /* get a pointer to elf structure */
    elf = elf_memory((char*)fileData,len);

    assert(elf != NULL);


    /* Get the first section */
    section = elf_getscn(elf, secidx);

    /* While there are no more sections */
    while (section != NULL)
    {
        gelf_getshdr(section, &shdr);


        if(shdr.sh_type == SHT_SYMTAB)
        {
            found = true;
            data = elf_getdata(section, NULL);
            count = shdr.sh_size / shdr.sh_entsize;
            DPRINTF(Loader, "Found Symbol Table, %d symbols present\n", count);

            /* loop through all the symbols, only loading global ones*/
            for (ii = 0; ii < count; ++ii)
            {
                gelf_getsym(data, ii, &sym);
                if (GELF_ST_BIND(sym.st_info) & STB_GLOBAL)
                {
                   symtab->insert(sym.st_value, elf_strptr(elf, shdr.sh_link, sym.st_name));
                }
            }
        }
        ++secidx;
        section = elf_getscn(elf, secidx);
    }

    elf_end(elf);

    return found;
}

bool
ElfObject::loadLocalSymbols(SymbolTable *symtab)
{

    Elf *elf;
    int secidx = 1; /* there is a 0 but it is nothing, go figure*/
    Elf_Scn *section;
    GElf_Shdr shdr;
    Elf_Data *data;
    int count, ii;
    bool found = false;
    GElf_Sym sym;

    if (!symtab)
        return false;

    /* check that header matches library version */
    assert(elf_version(EV_CURRENT) != EV_NONE);

    /* get a pointer to elf structure */
    elf = elf_memory((char*)fileData,len);

    assert(elf != NULL);


    /* Get the first section */
    section = elf_getscn(elf, secidx);

    /* While there are no more sections */
    while (section != NULL)
    {
        gelf_getshdr(section, &shdr);


        if(shdr.sh_type == SHT_SYMTAB)
        {
            found = true;
            data = elf_getdata(section, NULL);
            count = shdr.sh_size / shdr.sh_entsize;
            DPRINTF(Loader, "Found Symbol Table, %d symbols present\n", count);

            /* loop through all the symbols, only loading global ones*/
            for (ii = 0; ii < count; ++ii)
            {
                gelf_getsym(data, ii, &sym);
                if (GELF_ST_BIND(sym.st_info) & STB_LOCAL)
                {
                   symtab->insert(sym.st_value, elf_strptr(elf, shdr.sh_link, sym.st_name));
                }
            }
        }
        ++secidx;
        section = elf_getscn(elf, secidx);
    }

    elf_end(elf);

    return found;
}
