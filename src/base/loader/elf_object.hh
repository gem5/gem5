/*
 * Copyright (c) 2013 ARM Limited
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
 */

#ifndef __ELF_OBJECT_HH__
#define __ELF_OBJECT_HH__

#include <set>
#include <vector>

#include "base/loader/object_file.hh"

class ElfObject : public ObjectFile
{
  protected:
    // The global definition of a gem5 "Section" is closest to ELF's segments.
    typedef ObjectFile::Section Segment;

    // These values are provided to a linux process by the kernel, so we
    // need to keep them around.
    Addr _programHeaderTable;
    uint16_t _programHeaderSize;
    uint16_t _programHeaderCount;
    std::set<std::string> sectionNames;

    ElfObject *interpreter;

    // An interpreter load bias is the location in the process address space
    // where the interpreter is chosen to reside. Typically, this is carved
    // out of the top of the mmap reserve section.
    Addr ldBias;

    // The interpreter is typically a relocatable shared library and will
    // have a default value of zero which means that it does not care where
    // it is placed. However, the loader can be compiled and linked so that
    // it does care and needs a specific entry point.
    bool relocate;

    // The ldMin and ldMax fields are required to know how large of an
    // area is required to map the interpreter.
    Addr ldMin;
    Addr ldMax;

    /// Helper functions for loadGlobalSymbols() and loadLocalSymbols().
    bool loadSomeSymbols(SymbolTable *symtab, int binding, Addr mask,
                         Addr base, Addr offset);

    ElfObject(const std::string &_filename, size_t _len, uint8_t *_data,
              Arch _arch, OpSys _opSys);

    void getSections();
    bool sectionExists(std::string sec);

    std::vector<Segment> extraSegments;

  public:
    virtual ~ElfObject() {}

    bool loadSections(PortProxy& mem_proxy, Addr addr_mask = maxAddr,
                      Addr offset = 0) override;

    virtual bool loadAllSymbols(SymbolTable *symtab, Addr base = 0,
                                Addr offset = 0, Addr addr_mask = maxAddr)
                                override;

    virtual bool loadGlobalSymbols(SymbolTable *symtab, Addr base = 0,
                                   Addr offset = 0, Addr addr_mask = maxAddr)
                                   override;

    virtual bool loadLocalSymbols(SymbolTable *symtab, Addr base = 0,
                                  Addr offset = 0, Addr addr_mask = maxAddr)
                                  override;

    virtual bool loadWeakSymbols(SymbolTable *symtab, Addr base = 0,
                                 Addr offset = 0, Addr addr_mask = maxAddr)
                                 override;


    virtual ObjectFile *getInterpreter() const override
    { return interpreter; }
    virtual Addr bias() const override { return ldBias; }
    virtual bool relocatable() const override { return relocate; }
    virtual Addr mapSize() const override { return ldMax - ldMin; }
    virtual void updateBias(Addr bias_addr) override;

    virtual bool hasTLS() override { return sectionExists(".tbss"); }

    static ObjectFile *tryFile(const std::string &fname,
                               size_t len, uint8_t *data,
                               bool skip_interp_check = false);
    Addr programHeaderTable() {return _programHeaderTable;}
    uint16_t programHeaderSize() {return _programHeaderSize;}
    uint16_t programHeaderCount() {return _programHeaderCount;}
};

#endif // __ELF_OBJECT_HH__
