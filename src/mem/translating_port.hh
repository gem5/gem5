/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Ron Dreslinski
 *          Ali Saidi
 */

#ifndef __MEM_TRANSLATING_PROT_HH__
#define __MEM_TRANSLATING_PROT_HH__

#include "mem/port.hh"

class PageTable;
class Process;

class TranslatingPort : public FunctionalPort
{
  public:
    enum AllocType {
        Always,
        Never,
        NextPage
    };

  private:
    PageTable *pTable;
    Process *process;
    AllocType allocating;

  public:
    TranslatingPort(const std::string &_name,
                    Process *p, AllocType alloc);
    virtual ~TranslatingPort();

    bool tryReadBlob(Addr addr, uint8_t *p, int size);
    bool tryWriteBlob(Addr addr, uint8_t *p, int size);
    bool tryMemsetBlob(Addr addr, uint8_t val, int size);
    bool tryWriteString(Addr addr, const char *str);
    bool tryReadString(std::string &str, Addr addr);

    virtual void readBlob(Addr addr, uint8_t *p, int size);
    virtual void writeBlob(Addr addr, uint8_t *p, int size);
    virtual void memsetBlob(Addr addr, uint8_t val, int size);

    void writeString(Addr addr, const char *str);
    void readString(std::string &str, Addr addr);
};

#endif
