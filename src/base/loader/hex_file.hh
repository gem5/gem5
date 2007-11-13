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
 * Authors: Jaidev Patwardhan
 */

#ifndef __HEX_FILE_HH__
#define __HEX_FILE_HH__

#include <limits>
#include <string>

#include "sim/host.hh"	// for Addr
#include <fstream>

class Port;

class HexFile
{
  public:


  protected:
    const std::string filename;
    FILE *fp;

  public:
    virtual ~HexFile();
    HexFile(const std::string _filename);

    void close();

    bool loadSections(Port *memPort, Addr addrMask =
            std::numeric_limits<Addr>::max());

  protected:

  typedef struct {
    Addr MemAddr;
    uint32_t Data;
  } HexLine;

    Addr entry;
    Addr globalPtr;

  public:
    void parseLine(char *,Addr *,uint32_t *);
    Addr entryPoint() const { return entry; }
    Addr globalPointer() const { return globalPtr; }

};

#endif // __HEX_FILE_HH__
