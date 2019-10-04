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
 *
 * Authors: Steve Reinhardt
 */

#ifndef __BASE_LOADER_ECOFF_OBJECT_HH__
#define __BASE_LOADER_ECOFF_OBJECT_HH__

#include "base/loader/object_file.hh"

// forward decls: avoid including exec_ecoff.h here
struct ecoff_exechdr;
struct ecoff_filehdr;
struct ecoff_aouthdr;

class EcoffObjectFormat : public ObjectFileFormat
{
  public:
    ObjectFile *load(ImageFileDataPtr data) override;
};

class EcoffObject : public ObjectFile
{
  protected:
    const ecoff_exechdr *execHdr;
    const ecoff_filehdr *fileHdr;
    const ecoff_aouthdr *aoutHdr;

  public:
    EcoffObject(ImageFileDataPtr ifd);

    MemoryImage buildImage() const override;

    bool loadAllSymbols(SymbolTable *symtab, Addr base=0,
                        Addr offset=0, Addr addr_mask=MaxAddr) override;
    bool loadGlobalSymbols(SymbolTable *symtab, Addr base=0,
                           Addr offset=0, Addr addr_mask=MaxAddr) override;
    bool loadLocalSymbols(SymbolTable *symtab, Addr base=0,
                          Addr offset=0, Addr addr_mask=MaxAddr) override;
};

#endif // __BASE_LOADER_ECOFF_OBJECT_HH__
