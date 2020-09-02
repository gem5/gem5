/*
 * Copyright (c) 2011-2013, 2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#include "base/loader/elf_object.hh"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <string>

#include "base/bitfield.hh"
#include "base/loader/symtab.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Loader.hh"
#include "gelf.h"
#include "sim/byteswap.hh"

namespace Loader
{

ObjectFile *
ElfObjectFormat::load(ImageFileDataPtr ifd)
{
    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    ObjectFile *object = nullptr;

    // get a pointer to elf structure
    // Check that we actually have a elf file
    Elf *elf =
        elf_memory((char *)const_cast<uint8_t *>(ifd->data()), ifd->len());
    assert(elf);

    GElf_Ehdr ehdr;
    if (gelf_getehdr(elf, &ehdr) == 0)
        DPRINTFR(Loader, "Not ELF\n");
    else
        object = new ElfObject(ifd);

    elf_end(elf);

    return object;
}

namespace
{

ElfObjectFormat elfObjectFormat;
std::string interpDir;

} // anonymous namespace

void
setInterpDir(const std::string &dirname)
{
    fatal_if(!interpDir.empty(),
        "Error: setInterpDir has already been called once\n");
    interpDir = dirname;
}

ElfObject::ElfObject(ImageFileDataPtr ifd) : ObjectFile(ifd)
{
    // get a pointer to elf structure
    elf = elf_memory((char *)const_cast<uint8_t *>(imageData->data()),
                     imageData->len());
    assert(elf);
    gelf_getehdr(elf, &ehdr);

    determineArch();
    determineOpSys();

    entry = ehdr.e_entry;
    _programHeaderCount = ehdr.e_phnum;
    _programHeaderSize = ehdr.e_phentsize;

    // Go through all the segments in the program and record them.
    for (int i = 0; i < ehdr.e_phnum; ++i) {
        GElf_Phdr phdr;
        if (gelf_getphdr(elf, i, &phdr) == 0) {
            panic("gelf_getphdr failed for segment %d.", i);
        }

        if (phdr.p_type == PT_LOAD)
            handleLoadableSegment(phdr, i);
        if (phdr.p_type == PT_INTERP) {
            // Make sure the interpreter is an valid ELF file.
            auto interp_path = getInterpPath(phdr);
            ObjectFile *obj = createObjectFile(interp_path);
            interpreter = dynamic_cast<ElfObject *>(obj);
            assert(interpreter != nullptr);
            _symtab.insert(obj->symtab());
        }
    }

    // should have found at least one loadable segment
    warn_if(image.segments().empty(),
            "No loadable segments in '%s'. ELF file corrupted?\n",
            imageData->filename());

    for (auto M5_VAR_USED &seg: image.segments())
        DPRINTFR(Loader, "%s\n", seg);

    // We will actually read the sections when we need to load them

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // Get the first section
    int sec_idx = 1; // there is a 0 but it is nothing, go figure
    Elf_Scn *section = elf_getscn(elf, sec_idx);

    // While there are no more sections
    while (section) {
        GElf_Shdr shdr;
        gelf_getshdr(section, &shdr);

        if (shdr.sh_type == SHT_SYMTAB) {
            Elf_Data *data = elf_getdata(section, nullptr);
            int count = shdr.sh_size / shdr.sh_entsize;
            DPRINTF(Loader, "Found Symbol Table, %d symbols present.", count);

            // Loop through all the symbols.
            for (int i = 0; i < count; ++i) {
                GElf_Sym sym;
                gelf_getsym(data, i, &sym);

                char *sym_name = elf_strptr(elf, shdr.sh_link, sym.st_name);
                if (!sym_name || sym_name[0] == '$')
                    continue;

                Loader::Symbol symbol;
                symbol.address = sym.st_value;
                symbol.name = sym_name;

                switch (GELF_ST_BIND(sym.st_info)) {
                  case STB_GLOBAL:
                    symbol.binding = Loader::Symbol::Binding::Global;
                    break;
                  case STB_LOCAL:
                    symbol.binding = Loader::Symbol::Binding::Local;
                    break;
                  case STB_WEAK:
                    symbol.binding = Loader::Symbol::Binding::Weak;
                    break;
                  default:
                    continue;
                }

                if (_symtab.insert(symbol)) {
                    DPRINTF(Loader, "Symbol: %-40s value %#x.\n",
                            symbol.name, symbol.address);
                }
            }
        }
        ++sec_idx;
        section = elf_getscn(elf, sec_idx);
    }
}

std::string
ElfObject::getInterpPath(const GElf_Phdr &phdr) const
{
    // This is the interpreter path as specified in the elf file
    const std::string elf_path = (char *)imageData->data() + phdr.p_offset;
    if (!interpDir.empty())
        return interpDir + elf_path;
    else
        return elf_path;
}

void
ElfObject::determineArch()
{
    auto &emach = ehdr.e_machine;
    auto &eclass = ehdr.e_ident[EI_CLASS];
    auto &edata = ehdr.e_ident[EI_DATA];

    // Detect the architecture
    if (emach == EM_SPARC64 || (emach == EM_SPARC && eclass == ELFCLASS64) ||
        emach == EM_SPARCV9) {
        arch = SPARC64;
    } else if (emach == EM_SPARC32PLUS ||
               (emach == EM_SPARC && eclass == ELFCLASS32)) {
        arch = SPARC32;
    } else if (emach == EM_MIPS && eclass == ELFCLASS32) {
        arch = Mips;
        if (edata != ELFDATA2LSB) {
            fatal("The binary you're trying to load is compiled for big "
                  "endian MIPS. gem5\nonly supports little endian MIPS. "
                  "Please recompile your binary.\n");
        }
    } else if (emach == EM_X86_64 && eclass == ELFCLASS64) {
        arch = X86_64;
    } else if (emach == EM_386 && eclass == ELFCLASS32) {
        arch = I386;
    } else if (emach == EM_ARM && eclass == ELFCLASS32) {
        arch = bits(ehdr.e_entry, 0) ? Thumb : Arm;
    } else if (emach == EM_AARCH64 && eclass == ELFCLASS64) {
        arch = Arm64;
    } else if (emach == EM_RISCV) {
        arch = (eclass == ELFCLASS64) ? Riscv64 : Riscv32;
    } else if (emach == EM_PPC && eclass == ELFCLASS32) {
        arch = Power;
        if (edata != ELFDATA2MSB) {
            fatal("The binary you're trying to load is compiled for "
                  "little endian Power.\ngem5 only supports big "
                  "endian Power. Please recompile your binary.\n");
        }
    } else if (emach == EM_PPC64) {
        fatal("The binary you're trying to load is compiled for 64-bit "
              "Power. M5\n only supports 32-bit Power. Please "
              "recompile your binary.\n");
    } else {
        warn("Unknown architecture: %d\n", emach);
    }
}

void
ElfObject::determineOpSys()
{
    // Detect the operating system
    switch (ehdr.e_ident[EI_OSABI]) {
      case ELFOSABI_LINUX:
        opSys = Linux;
        return;
      case ELFOSABI_SOLARIS:
        opSys = Solaris;
        return;
      case ELFOSABI_TRU64:
        opSys = Tru64;
        return;
      case ELFOSABI_ARM:
        opSys = LinuxArmOABI;
        return;
      case ELFOSABI_FREEBSD:
        opSys = FreeBSD;
        return;
      default:
        opSys = UnknownOpSys;
    }

    Elf_Scn *section = elf_getscn(elf, 1);
    for (int sec_idx = 1; section; section = elf_getscn(elf, ++sec_idx)) {
        GElf_Shdr shdr;
        gelf_getshdr(section, &shdr);

        char *e_str = elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name);
        if (shdr.sh_type == SHT_NOTE && !strcmp(".note.ABI-tag", e_str)) {
            // we have found a ABI note section
            // Check the 5th 32bit word for OS  0 == linux, 1 == hurd,
            // 2 == solaris, 3 == freebsd
            Elf_Data *raw_data = elf_rawdata(section, nullptr);
            assert(raw_data && raw_data->d_buf);

            uint32_t raw_abi = ((uint32_t *)raw_data->d_buf)[4];
            bool is_le = ehdr.e_ident[EI_DATA] == ELFDATA2LSB;
            uint32_t os_abi = is_le ? htole(raw_abi) : htobe(raw_abi);

            switch (os_abi) {
              case 0:
                opSys = Linux;
                return;
              case 1:
                fatal("gem5 does not support the HURD ABI.\n");
              case 2:
                opSys = Solaris;
                return;
              case 3:
                opSys = FreeBSD;
                return;
            }
        }

        if (!strcmp(".SUNW_version", e_str) || !strcmp(".stab.index", e_str)) {
            opSys = Solaris;
            return;
        }
    }
}

void
ElfObject::handleLoadableSegment(GElf_Phdr phdr, int seg_num)
{
    auto name = std::to_string(seg_num);

    image.addSegment({ name, phdr.p_paddr, imageData,
                       phdr.p_offset, phdr.p_filesz });
    Addr uninitialized = phdr.p_memsz - phdr.p_filesz;
    if (uninitialized) {
        // There may be parts of a segment which aren't included in the
        // file. In those cases, we need to create a new segment with no
        // data to take up the extra space. This should be zeroed when
        // loaded into memory.
        image.addSegment({ name + "(uninitialized)",
                           phdr.p_paddr + phdr.p_filesz, uninitialized });
    }

    const Addr file_start = phdr.p_offset;
    const Addr file_end = file_start + phdr.p_filesz;

    // If there is a program header table, figure out the virtual
    // address of the header table in the final memory image. We use
    // the program headers themselves to translate from a file offset
    // to the address in the image.
    if (file_start <= ehdr.e_phoff && file_end > ehdr.e_phoff)
        _programHeaderTable = phdr.p_vaddr + (ehdr.e_phoff - file_start);
}

ElfObject::~ElfObject()
{
    elf_end(elf);
}

void
ElfObject::getSections()
{
    assert(!sectionNames.size());

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    Elf *elf =
        elf_memory((char *)const_cast<uint8_t *>(imageData->data()),
                imageData->len());
    assert(elf != NULL);

    // Check that we actually have a elf file
    GElf_Ehdr ehdr;
    if (gelf_getehdr(elf, &ehdr) ==0) {
        panic("Not ELF, shouldn't be here");
    }

    // Get the first section
    int sec_idx = 1; // there is a 0 but it is nothing, go figure
    Elf_Scn *section = elf_getscn(elf, sec_idx);

    // While there are no more sections
    while (section) {
        GElf_Shdr shdr;
        gelf_getshdr(section, &shdr);
        sectionNames.insert(elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name));
        section = elf_getscn(elf, ++sec_idx);
    } // while sections

    elf_end(elf);
}

bool
ElfObject::sectionExists(std::string sec)
{
    if (!sectionNames.size())
        getSections();

    return sectionNames.find(sec) != sectionNames.end();
}


void
ElfObject::updateBias(Addr bias_addr)
{
    // Record the bias.
    ldBias = bias_addr;

    // Patch the entry point with bias_addr.
    entry += bias_addr;

    // Patch segments with the bias_addr.
    image.offset(bias_addr);
}

} // namespace Loader
