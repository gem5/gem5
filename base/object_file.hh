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

#ifndef __OBJECT_FILE_HH__
#define __OBJECT_FILE_HH__

#include "ecoff.hh"
#include "isa_traits.hh"	// for Addr

class SymbolTable;

class ObjectFile
{
  protected:
    std::string name;
    int descriptor;
    uint8_t *data;
    size_t len;

  public:
    ObjectFile();
    explicit ObjectFile(std::string file);
    virtual ~ObjectFile();

    bool open(std::string file);
    void close();

    virtual bool loadGlobals(SymbolTable *symtab) = 0;
    virtual bool loadLocals(SymbolTable *symtab) = 0;
    virtual void postOpen() = 0;

  protected:
    Addr text_off;
    Addr data_off;
    Addr bss_off;

    size_t text_size;
    size_t data_size;
    size_t bss_size;

  public:
    Addr textOffset() const { return text_off; }
    Addr dataOffset() const { return data_off; }
    Addr bssOffset() const { return bss_off; }

    size_t textSize() const { return text_size; }
    size_t dataSize() const { return data_size; }
    size_t bssSize() const { return bss_size; }
};

class EcoffObject : public ObjectFile
{
  protected:
    EcoffFileHeader *exec;
    EcoffAOutHeader *aout;

  public:
    EcoffObject() {}
    explicit EcoffObject(std::string file) { open(file); }
    virtual ~EcoffObject() {}

    virtual bool loadGlobals(SymbolTable *symtab);
    virtual bool loadLocals(SymbolTable *symtab);
    virtual void postOpen();
};

#endif // __OBJECT_FILE_HH__
