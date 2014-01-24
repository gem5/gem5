/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __OBJECT_FILE_HH__
#define __OBJECT_FILE_HH__

#include <limits>
#include <string>

#include "base/types.hh"

class PortProxy;
class SymbolTable;

class ObjectFile
{
  public:

    enum Arch {
        UnknownArch,
        Alpha,
        SPARC64,
        SPARC32,
        Mips,
        X86_64,
        I386,
        Arm64,
        Arm,
        Thumb,
        Power
    };

    enum OpSys {
        UnknownOpSys,
        Tru64,
        Linux,
        Solaris,
        LinuxArmOABI
    };

  protected:
    const std::string filename;
    int descriptor;
    uint8_t *fileData;
    size_t len;

    Arch  arch;
    OpSys opSys;

    ObjectFile(const std::string &_filename, int _fd,
               size_t _len, uint8_t *_data,
               Arch _arch, OpSys _opSys);

  public:
    virtual ~ObjectFile();

    void close();

    virtual bool loadSections(PortProxy& memProxy, Addr addrMask =
                              std::numeric_limits<Addr>::max(),
                              Addr offset = 0);
    virtual bool loadGlobalSymbols(SymbolTable *symtab, Addr addrMask =
            std::numeric_limits<Addr>::max()) = 0;
    virtual bool loadLocalSymbols(SymbolTable *symtab, Addr addrMask =
            std::numeric_limits<Addr>::max()) = 0;
    virtual bool loadWeakSymbols(SymbolTable *symtab, Addr addrMask =
            std::numeric_limits<Addr>::max())
    { return false; }

    virtual bool isDynamic() { return false; }
    virtual bool hasTLS() { return false; }

    Arch  getArch()  const { return arch; }
    OpSys getOpSys() const { return opSys; }

  protected:

    struct Section {
        Addr     baseAddr;
        uint8_t *fileImage;
        size_t   size;
    };

    Addr entry;
    Addr globalPtr;

    Section text;
    Section data;
    Section bss;

    bool loadSection(Section *sec, PortProxy& memProxy, Addr addrMask,
                     Addr offset = 0);
    void setGlobalPointer(Addr global_ptr) { globalPtr = global_ptr; }

  public:
    Addr entryPoint() const { return entry; }

    Addr globalPointer() const { return globalPtr; }

    Addr textBase() const { return text.baseAddr; }
    Addr dataBase() const { return data.baseAddr; }
    Addr bssBase() const { return bss.baseAddr; }

    size_t textSize() const { return text.size; }
    size_t dataSize() const { return data.size; }
    size_t bssSize() const { return bss.size; }

    /* This function allows you to override the base address where
     * a binary is going to be loaded or set it if the binary is just a
     * blob that doesn't include an object header.
     * @param a address to load the binary/text section at
     */
    void setTextBase(Addr a) { text.baseAddr = a; }
};

ObjectFile *createObjectFile(const std::string &fname, bool raw = false);


#endif // __OBJECT_FILE_HH__
