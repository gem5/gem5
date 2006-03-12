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
 */

#include <string>
#include "base/chunk_generator.hh"
#include "mem/port.hh"
#include "mem/translating_port.hh"
#include "mem/page_table.hh"

using namespace TheISA;

TranslatingPort::TranslatingPort(Port *_port, PageTable *p_table)
    : port(_port), pTable(p_table)
{ }

TranslatingPort::~TranslatingPort()
{ }

bool
TranslatingPort::tryReadBlobFunctional(Addr addr, uint8_t *p, int size)
{
    Addr paddr;
    int prevSize = 0;

    for (ChunkGenerator gen(addr, size, VMPageSize); !gen.done(); gen.next()) {

        if (!pTable->translate(gen.addr(),paddr))
            return false;

        port->readBlobFunctional(paddr, p + prevSize, gen.size());
        prevSize += gen.size();
    }

    return true;
}

void
TranslatingPort::readBlobFunctional(Addr addr, uint8_t *p, int size)
{
    if (!tryReadBlobFunctional(addr, p, size))
        fatal("readBlobFunctional(0x%x, ...) failed", addr);
}


bool
TranslatingPort::tryWriteBlobFunctional(Addr addr, uint8_t *p, int size,
                                        bool alloc)
{

    Addr paddr;
    int prevSize = 0;

    for (ChunkGenerator gen(addr, size, VMPageSize); !gen.done(); gen.next()) {

        if (!pTable->translate(gen.addr(), paddr)) {
            if (alloc) {
                pTable->allocate(roundDown(gen.addr(), VMPageSize),
                                 VMPageSize);
                pTable->translate(gen.addr(), paddr);
            } else {
                return false;
            }
        }

        port->writeBlobFunctional(paddr, p + prevSize, gen.size());
        prevSize += gen.size();
    }

    return true;
}


void
TranslatingPort::writeBlobFunctional(Addr addr, uint8_t *p, int size,
                                     bool alloc)
{
    if (!tryWriteBlobFunctional(addr, p, size, alloc))
        fatal("writeBlobFunctional(0x%x, ...) failed", addr);
}

bool
TranslatingPort::tryMemsetBlobFunctional(Addr addr, uint8_t val, int size,
                                         bool alloc)
{
    Addr paddr;

    for (ChunkGenerator gen(addr, size, VMPageSize); !gen.done(); gen.next()) {

        if (!pTable->translate(gen.addr(), paddr)) {
            if (alloc) {
                pTable->allocate(roundDown(gen.addr(), VMPageSize),
                                 VMPageSize);
                pTable->translate(gen.addr(), paddr);
            } else {
                return false;
            }
        }

        port->memsetBlobFunctional(paddr, val, gen.size());
    }

    return true;
}

void
TranslatingPort::memsetBlobFunctional(Addr addr, uint8_t val, int size,
                                      bool alloc)
{
    if (!tryMemsetBlobFunctional(addr, val, size, alloc))
        fatal("memsetBlobFunctional(0x%x, ...) failed", addr);
}


bool
TranslatingPort::tryWriteStringFunctional(Addr addr, const char *str)
{
    Addr paddr,vaddr;
    uint8_t c;

    vaddr = addr;

    do {
        c = *str++;
        if (!pTable->translate(vaddr++,paddr))
            return false;

        port->writeBlobFunctional(paddr, &c, 1);
    } while (c);

    return true;
}

void
TranslatingPort::writeStringFunctional(Addr addr, const char *str)
{
    if (!tryWriteStringFunctional(addr, str))
        fatal("writeStringFunctional(0x%x, ...) failed", addr);
}

bool
TranslatingPort::tryReadStringFunctional(std::string &str, Addr addr)
{
    Addr paddr,vaddr;
    uint8_t c;

    vaddr = addr;

    do {
        if (!pTable->translate(vaddr++,paddr))
            return false;

        port->readBlobFunctional(paddr, &c, 1);
        str += c;
    } while (c);

    return true;
}

void
TranslatingPort::readStringFunctional(std::string &str, Addr addr)
{
    if (!tryReadStringFunctional(str, addr))
        fatal("readStringFunctional(0x%x, ...) failed", addr);
}

