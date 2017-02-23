/*
 * Copyright (c) 2011-2013 ARM Limited
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
 *
 * Authors: Steve Reinhardt
 *          Ali Saidi
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
#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/Loader.hh"
#include "gelf.h"
#include "sim/byteswap.hh"

ObjectFile *
ElfObject::tryFile(const std::string &fname, size_t len, uint8_t *data,
                   bool skip_interp_check)
{
    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    // Check that we actually have a elf file
    Elf *elf = elf_memory((char*)data, len);
    assert(elf);

    GElf_Ehdr ehdr;
    if (gelf_getehdr(elf, &ehdr) == 0) {
        DPRINTFR(Loader, "Not ELF\n");
        elf_end(elf);
        return NULL;
    }

    // Detect the architecture
    Arch arch = UnknownArch;
    if (ehdr.e_machine == EM_SPARC64 ||
        (ehdr.e_machine == EM_SPARC &&
         ehdr.e_ident[EI_CLASS] == ELFCLASS64) ||
        ehdr.e_machine == EM_SPARCV9) {
        arch = SPARC64;
    } else if (ehdr.e_machine == EM_SPARC32PLUS ||
               (ehdr.e_machine == EM_SPARC &&
                ehdr.e_ident[EI_CLASS] == ELFCLASS32)) {
        arch = SPARC32;
    } else if (ehdr.e_machine == EM_MIPS &&
               ehdr.e_ident[EI_CLASS] == ELFCLASS32) {
        arch = Mips;
        if (ehdr.e_ident[EI_DATA] != ELFDATA2LSB) {
            fatal("The binary you're trying to load is compiled for big "
                  "endian MIPS. gem5\nonly supports little endian MIPS. "
                  "Please recompile your binary.\n");
        }
    } else if (ehdr.e_machine == EM_X86_64 &&
               ehdr.e_ident[EI_CLASS] == ELFCLASS64) {
        arch = X86_64;
    } else if (ehdr.e_machine == EM_386 &&
               ehdr.e_ident[EI_CLASS] == ELFCLASS32) {
        arch = I386;
    } else if (ehdr.e_machine == EM_ARM &&
               ehdr.e_ident[EI_CLASS] == ELFCLASS32) {
        arch = bits(ehdr.e_entry, 0) ? Thumb : Arm;
    } else if (ehdr.e_machine == EM_AARCH64 &&
               ehdr.e_ident[EI_CLASS] == ELFCLASS64) {
        arch = Arm64;
    } else if (ehdr.e_machine == EM_RISCV) {
        arch = Riscv;
    } else if (ehdr.e_machine == EM_PPC &&
               ehdr.e_ident[EI_CLASS] == ELFCLASS32) {
        arch = Power;
        if (ehdr.e_ident[EI_DATA] != ELFDATA2MSB) {
            fatal("The binary you're trying to load is compiled for "
                  "little endian Power.\ngem5 only supports big "
                  "endian Power. Please recompile your binary.\n");
        }
    } else if (ehdr.e_machine == EM_PPC64) {
        fatal("The binary you're trying to load is compiled for 64-bit "
              "Power. M5\n only supports 32-bit Power. Please "
              "recompile your binary.\n");
    } else if (ehdr.e_ident[EI_CLASS] == ELFCLASS64) {
        // Since we don't know how to check for alpha right now, we'll
        // just assume if it wasn't something else and it's 64 bit, that's
        // what it must be.
        arch = Alpha;
    } else {
        warn("Unknown architecture: %d\n", ehdr.e_machine);
        arch = UnknownArch;
    }

    // Detect the operating system
    OpSys op_sys;
    switch (ehdr.e_ident[EI_OSABI]) {
      case ELFOSABI_LINUX:
        op_sys = Linux;
        break;
      case ELFOSABI_SOLARIS:
        op_sys = Solaris;
        break;
      case ELFOSABI_TRU64:
        op_sys = Tru64;
        break;
      case ELFOSABI_ARM:
        op_sys = LinuxArmOABI;
        break;
      case ELFOSABI_FREEBSD:
        op_sys = FreeBSD;
        break;
      default:
        op_sys = UnknownOpSys;
    }

    // Take a look at the .note.ABI section.
    // It can let us know what's what.
    if (op_sys == UnknownOpSys) {
        int sec_idx = 1;

        // Get the first section
        Elf_Scn *section = elf_getscn(elf, sec_idx);

        // While there are no more sections
        while (section && op_sys == UnknownOpSys) {
            GElf_Shdr shdr;
            gelf_getshdr(section, &shdr);

            char *e_str = elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name);
            if (shdr.sh_type == SHT_NOTE &&
                !strcmp(".note.ABI-tag", e_str)) {
                // we have found a ABI note section
                // Check the 5th 32bit word for OS  0 == linux, 1 == hurd,
                // 2 == solaris, 3 == freebsd
                Elf_Data *raw_data = elf_rawdata(section, NULL);
                assert(raw_data && raw_data->d_buf);

                uint32_t raw_abi = ((uint32_t*)raw_data->d_buf)[4];
                bool is_le = ehdr.e_ident[EI_DATA] == ELFDATA2LSB;
                uint32_t os_abi = is_le ? htole(raw_abi) : htobe(raw_abi);

                switch (os_abi) {
                  case 0:
                    op_sys = Linux;
                    break;
                  case 1:
                    fatal("gem5 does not support the HURD ABI.\n");
                  case 2:
                    op_sys = Solaris;
                    break;
                  case 3:
                    op_sys = FreeBSD;
                    break;
                }
            } // if section found

            if (!strcmp(".SUNW_version", e_str) ||
                !strcmp(".stab.index", e_str))
                op_sys = Solaris;

            section = elf_getscn(elf, ++sec_idx);
        } // while sections
    }

    ElfObject * result = new ElfObject(fname, len, data, arch, op_sys);

    // The number of headers in the file
    result->_programHeaderCount = ehdr.e_phnum;
    // Record the size of each entry
    result->_programHeaderSize = ehdr.e_phentsize;
    result->_programHeaderTable = 0;
    if (result->_programHeaderCount) { // If there is a program header table
        // Figure out the virtual address of the header table in the
        // final memory image. We use the program headers themselves
        // to translate from a file offset to the address in the image.
        GElf_Phdr phdr;
        uint64_t e_phoff = ehdr.e_phoff;

        for (int i = 0; i < result->_programHeaderCount; i++) {
            gelf_getphdr(elf, i, &phdr);
            // Check if we've found the segment with the headers in it
            if (phdr.p_offset <= e_phoff &&
                phdr.p_offset + phdr.p_filesz > e_phoff) {
                result->_programHeaderTable =
                    phdr.p_paddr + (e_phoff - phdr.p_offset);
                break;
            }
        }
    }

    if (!skip_interp_check) {
        for (int i = 0; i < ehdr.e_phnum; i++) {
            GElf_Phdr phdr;
            M5_VAR_USED void *check_p = gelf_getphdr(elf, i, &phdr);
            assert(check_p != nullptr);

           if (phdr.p_type != PT_INTERP)
                continue;

            char *interp_path = (char*)data + phdr.p_offset;
            int fd = open(interp_path, O_RDONLY);
            if (fd == -1)
                fatal("Unable to open dynamic executable's interpreter.\n");

            struct stat sb;
            M5_VAR_USED int check_i = fstat(fd, &sb);
            assert(check_i == 0);

            void *mm = mmap(nullptr, sb.st_size, PROT_READ,
                            MAP_PRIVATE, fd, 0);
            assert(mm != MAP_FAILED);
            close(fd);

            uint8_t *interp_image = (uint8_t*)mm;
            ObjectFile *obj = tryFile(interp_path, sb.st_size,
                                      interp_image, true);
            assert(obj != nullptr);
            result->interpreter = dynamic_cast<ElfObject*>(obj);
            assert(result->interpreter != nullptr);
            break;
        }
    }

    elf_end(elf);
    return result;
}

ElfObject::ElfObject(const std::string &_filename, size_t _len,
                     uint8_t *_data, Arch _arch, OpSys _op_sys)
    : ObjectFile(_filename, _len, _data, _arch, _op_sys),
      _programHeaderTable(0), _programHeaderSize(0), _programHeaderCount(0),
      interpreter(nullptr), ldBias(0), relocate(true),
      ldMin(std::numeric_limits<Addr>::max()),
      ldMax(std::numeric_limits<Addr>::min())
{
    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    Elf *elf = elf_memory((char*)fileData,len);
    assert(elf);

    // Check that we actually have a elf file
    GElf_Ehdr ehdr;
    if (gelf_getehdr(elf, &ehdr) ==0) {
        panic("Not ELF, shouldn't be here");
    }

    entry = ehdr.e_entry;

    // initialize segment sizes to 0 in case they're not present
    text.size = data.size = bss.size = 0;
    text.baseAddr = data.baseAddr = bss.baseAddr = 0;

    int sec_idx = 1;

    // The first address of some important sections.
    Addr text_sec_start = 0;
    Addr data_sec_start = 0;
    Addr bss_sec_start = 0;

    // Get the first section
    Elf_Scn *section = elf_getscn(elf, sec_idx);

    // Find the beginning of the most interesting sections.
    while (section) {
        GElf_Shdr shdr;
        gelf_getshdr(section, &shdr);
        char *sec_name = elf_strptr(elf, ehdr.e_shstrndx, shdr.sh_name);

        if (sec_name) {
            if (!strcmp(".text", sec_name)) {
                text_sec_start = shdr.sh_addr;
            } else if (!strcmp(".data", sec_name)) {
                data_sec_start = shdr.sh_addr;
            } else if (!strcmp(".bss", sec_name)) {
                bss_sec_start = shdr.sh_addr;
            }
        } else {
            Elf_Error errorNum = (Elf_Error)elf_errno();
            if (errorNum != ELF_E_NONE) {
                const char *errorMessage = elf_errmsg(errorNum);
                fatal("Error from libelf: %s.\n", errorMessage);
            }
        }

        section = elf_getscn(elf, ++sec_idx);
    }

    // Go through all the segments in the program, record them, and scrape
    // out information about the text, data, and bss areas needed by other
    // code.
    for (int i = 0; i < ehdr.e_phnum; ++i) {
        GElf_Phdr phdr;
        if (gelf_getphdr(elf, i, &phdr) == 0) {
            panic("gelf_getphdr failed for segment %d.", i);
        }

        // for now we don't care about non-loadable segments
        if (!(phdr.p_type & PT_LOAD))
            continue;

        ldMin = std::min(ldMin, phdr.p_vaddr);
        ldMax = std::max(ldMax, phdr.p_vaddr + phdr.p_memsz);

        // Check to see if this segment contains the bss section.
        if (phdr.p_paddr <= bss_sec_start &&
            phdr.p_paddr + phdr.p_memsz > bss_sec_start &&
            phdr.p_memsz - phdr.p_filesz > 0) {
            bss.baseAddr = phdr.p_paddr + phdr.p_filesz;
            bss.size = phdr.p_memsz - phdr.p_filesz;
            bss.fileImage = NULL;
        }

        // Check to see if this is the text or data segment
        if (phdr.p_vaddr <= text_sec_start &&
            phdr.p_vaddr + phdr.p_filesz > text_sec_start) {

            // If this value is nonzero, we need to flip the relocate flag.
            if (phdr.p_vaddr != 0)
                relocate = false;

            text.baseAddr = phdr.p_paddr;
            text.size = phdr.p_filesz;
            text.fileImage = fileData + phdr.p_offset;
        } else if (phdr.p_vaddr <= data_sec_start &&
                   phdr.p_vaddr + phdr.p_filesz > data_sec_start) {
            data.baseAddr = phdr.p_paddr;
            data.size = phdr.p_filesz;
            data.fileImage = fileData + phdr.p_offset;
        } else {
            // If it's none of the above but is loadable,
            // load the filesize worth of data
            Segment extra;
            extra.baseAddr = phdr.p_paddr;
            extra.size = phdr.p_filesz;
            extra.fileImage = fileData + phdr.p_offset;
            extraSegments.push_back(extra);
        }
    }

    // should have found at least one loadable segment
    warn_if(text.size == 0,
            "Empty .text segment in '%s'. ELF file corrupted?\n",
            filename);

    DPRINTFR(Loader, "text: 0x%x %d\ndata: 0x%x %d\nbss: 0x%x %d\n",
             text.baseAddr, text.size, data.baseAddr, data.size,
             bss.baseAddr, bss.size);

    elf_end(elf);

    // We will actually read the sections when we need to load them
}


bool
ElfObject::loadSomeSymbols(SymbolTable *symtab, int binding, Addr mask,
                           Addr base, Addr offset)
{
    if (!symtab)
        return false;

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    Elf *elf = elf_memory((char*)fileData,len);
    assert(elf != NULL);

    // Get the first section
    int sec_idx = 1; // there is a 0 but it is nothing, go figure
    Elf_Scn *section = elf_getscn(elf, sec_idx);

    // While there are no more sections
    bool found = false;
    while (section != NULL) {
        GElf_Shdr shdr;
        gelf_getshdr(section, &shdr);

        if (shdr.sh_type == SHT_SYMTAB) {
            found = true;
            Elf_Data *data = elf_getdata(section, NULL);
            int count = shdr.sh_size / shdr.sh_entsize;
            DPRINTF(Loader, "Found Symbol Table, %d symbols present\n", count);

            // loop through all the symbols, only loading global ones
            for (int i = 0; i < count; ++i) {
                GElf_Sym sym;
                gelf_getsym(data, i, &sym);
                if (GELF_ST_BIND(sym.st_info) == binding) {
                    char *sym_name = elf_strptr(elf, shdr.sh_link, sym.st_name);
                    if (sym_name && sym_name[0] != '$') {
                        Addr value = sym.st_value - base + offset;
                        if (symtab->insert(value & mask, sym_name)) {
                            DPRINTF(Loader, "Symbol: %-40s value %#x\n",
                                    sym_name, value);
                        }
                    }
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
ElfObject::loadAllSymbols(SymbolTable *symtab, Addr base, Addr offset,
                          Addr addr_mask)
{
    return (loadGlobalSymbols(symtab, base, offset, addr_mask) &&
            loadLocalSymbols(symtab, base, offset, addr_mask) &&
            loadWeakSymbols(symtab, base, offset, addr_mask));
}

bool
ElfObject::loadGlobalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                             Addr addr_mask)
{
    if (interpreter) {
        interpreter->loadSomeSymbols(symtab, STB_GLOBAL, addr_mask,
                                     base, offset);
    }
    return loadSomeSymbols(symtab, STB_GLOBAL, addr_mask, base, offset);
}

bool
ElfObject::loadLocalSymbols(SymbolTable *symtab, Addr base, Addr offset,
                            Addr addr_mask)
{
    if (interpreter) {
        interpreter->loadSomeSymbols(symtab, STB_LOCAL, addr_mask,
                                     base, offset);
    }
    return loadSomeSymbols(symtab, STB_LOCAL, addr_mask, base, offset);
}

bool
ElfObject::loadWeakSymbols(SymbolTable *symtab, Addr base, Addr offset,
                           Addr addr_mask)
{
    if (interpreter) {
        interpreter->loadSomeSymbols(symtab, STB_WEAK, addr_mask,
                                     base, offset);
    }
    return loadSomeSymbols(symtab, STB_WEAK, addr_mask, base, offset);
}

bool
ElfObject::loadSections(PortProxy& mem_proxy, Addr addr_mask, Addr offset)
{
    if (!ObjectFile::loadSections(mem_proxy, addr_mask, offset))
        return false;

    for (auto seg : extraSegments) {
        if (!loadSection(&seg, mem_proxy, addr_mask, offset)) {
            return false;
        }
    }

    if (interpreter)
        interpreter->loadSections(mem_proxy, addr_mask, offset);

    return true;
}

void
ElfObject::getSections()
{
    assert(!sectionNames.size());

    // check that header matches library version
    if (elf_version(EV_CURRENT) == EV_NONE)
        panic("wrong elf version number!");

    // get a pointer to elf structure
    Elf *elf = elf_memory((char*)fileData,len);
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
    text.baseAddr += bias_addr;
    data.baseAddr += bias_addr;
    bss.baseAddr  += bias_addr;
    for (auto &segment : extraSegments)
        segment.baseAddr += bias_addr;
}
