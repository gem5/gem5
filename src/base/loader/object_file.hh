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
 */

#ifndef __BASE_LOADER_OBJECT_FILE_HH__
#define __BASE_LOADER_OBJECT_FILE_HH__

#include <string>

#include "base/loader/image_file.hh"
#include "base/loader/image_file_data.hh"
#include "base/loader/memory_image.hh"
#include "base/logging.hh"
#include "base/types.hh"

namespace Loader
{

enum Arch {
    UnknownArch,
    SPARC64,
    SPARC32,
    Mips,
    X86_64,
    I386,
    Arm64,
    Arm,
    Thumb,
    Power,
    Riscv64,
    Riscv32
};

enum OpSys {
    UnknownOpSys,
    Tru64,
    Linux,
    Solaris,
    LinuxArmOABI,
    FreeBSD
};

class SymbolTable;

class ObjectFile : public ImageFile
{
  protected:
    Arch arch = UnknownArch;
    OpSys opSys = UnknownOpSys;

    ObjectFile(ImageFileDataPtr ifd);

  public:
    virtual ~ObjectFile() {};

    virtual bool
    loadAllSymbols(SymbolTable *symtab, Addr base=0,
            Addr offset=0, Addr mask=MaxAddr)
    {
        return true;
    };
    virtual bool
    loadGlobalSymbols(SymbolTable *symtab, Addr base=0,
                      Addr offset=0, Addr mask=MaxAddr)
    {
        return true;
    }
    virtual bool
    loadLocalSymbols(SymbolTable *symtab, Addr base=0,
                     Addr offset=0, Addr mask=MaxAddr)
    {
        return true;
    }
    virtual bool
    loadWeakSymbols(SymbolTable *symtab, Addr base=0,
                    Addr offset=0, Addr mask=MaxAddr)
    {
        return true;
    }

    virtual ObjectFile *getInterpreter() const { return nullptr; }
    virtual bool relocatable() const { return false; }
    virtual Addr
    mapSize() const
    {
        panic("mapSize() should only be called on relocatable objects\n");
    }
    virtual void
    updateBias(Addr bias_addr)
    {
        panic("updateBias() should only be called on relocatable objects\n");
    }
    virtual Addr bias() const { return 0; }

    virtual bool hasTLS() { return false; }

    Arch  getArch()  const { return arch; }
    OpSys getOpSys() const { return opSys; }

  protected:
    Addr entry = 0;

  public:
    Addr entryPoint() const { return entry; }
};

class ObjectFileFormat
{
  protected:
    ObjectFileFormat();

  public:
    ObjectFileFormat(const ObjectFileFormat &) = delete;
    void operator=(const ObjectFileFormat &) = delete;

    virtual ObjectFile *load(ImageFileDataPtr data) = 0;
};

ObjectFile *createObjectFile(const std::string &fname, bool raw=false);

} // namespace Loader

#endif // __BASE_LOADER_OBJECT_FILE_HH__
