/*
 * Copyright (c) 2013, 2019 ARM Limited
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

#ifndef __BASE_LOADER_ELF_OBJECT_HH__
#define __BASE_LOADER_ELF_OBJECT_HH__

#include <set>
#include <vector>

#include "base/compiler.hh"
#include "base/loader/object_file.hh"
#include "gelf.h"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Loader, loader);
namespace loader
{

class ElfObjectFormat : public ObjectFileFormat
{
  public:
    ObjectFile *load(ImageFileDataPtr data) override;
};

class ElfObject : public ObjectFile
{
  protected:
    Elf *elf;
    GElf_Ehdr ehdr;

    void determineArch();
    void determineOpSys();
    void determineByteOrder();
    void handleLoadableSegment(GElf_Phdr phdr, int seg_num);

    // These values are provided to a linux process by the kernel, so we
    // need to keep them around.
    Addr _programHeaderTable = 0;
    uint16_t _programHeaderSize = 0;
    uint16_t _programHeaderCount = 0;
    std::set<std::string> sectionNames;

    ElfObject *interpreter = nullptr;

    // An interpreter load bias is the location in the process address space
    // where the interpreter is chosen to reside. Typically, this is carved
    // out of the top of the mmap reserve section.
    Addr ldBias = 0;

    // The interpreter is typically a relocatable shared library and will
    // have a default value of zero which means that it does not care where
    // it is placed. However, the loader can be compiled and linked so that
    // it does care and needs a specific entry point.
    bool relocate = true;

    // The ldMin and ldMax fields are required to know how large of an
    // area is required to map the interpreter.
    Addr ldMin = MaxAddr;
    Addr ldMax = MaxAddr;

    /// Helper functions for loadGlobalSymbols() and loadLocalSymbols().
    bool loadSomeSymbols(SymbolTable *symtab, int binding, Addr mask,
                         Addr base, Addr offset);

    void getSections();
    bool sectionExists(std::string sec);

    MemoryImage image;

  public:
    ElfObject(ImageFileDataPtr ifd);
    ~ElfObject();

    MemoryImage buildImage() const override { return image; }

    ObjectFile *getInterpreter() const override { return interpreter; }
    std::string getInterpPath(const GElf_Phdr &phdr) const;

    Addr bias() const override { return ldBias; }
    bool relocatable() const override { return relocate; }
    Addr mapSize() const override { return ldMax - ldMin; }
    void updateBias(Addr bias_addr) override;

    bool hasTLS() override { return sectionExists(".tbss"); }

    Addr programHeaderTable() {return _programHeaderTable;}
    uint16_t programHeaderSize() {return _programHeaderSize;}
    uint16_t programHeaderCount() {return _programHeaderCount;}
};

/**
 * This is the interface for setting up a base path for the
 * elf interpreter. This is needed when loading a
 * cross-compiled (guest ISA != host ISA) dynamically
 * linked application.
 * @param dirname base path for the interpreter
 */
void setInterpDir(const std::string &dirname);

} // namespace loader
} // namespace gem5

#endif // __BASE_LOADER_ELF_OBJECT_HH__
