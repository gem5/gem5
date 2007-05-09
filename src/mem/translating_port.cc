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
 *          Steve Reinhardt
 */

#include <string>
#include "base/chunk_generator.hh"
#include "mem/port.hh"
#include "mem/translating_port.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"

using namespace TheISA;

TranslatingPort::TranslatingPort(const std::string &_name,
                                 Process *p, AllocType alloc)
    : FunctionalPort(_name), pTable(p->pTable), process(p),
      allocating(alloc)
{ }

TranslatingPort::~TranslatingPort()
{ }

bool
TranslatingPort::tryReadBlob(Addr addr, uint8_t *p, int size)
{
    Addr paddr;
    int prevSize = 0;

    for (ChunkGenerator gen(addr, size, VMPageSize); !gen.done(); gen.next()) {

        if (!pTable->translate(gen.addr(),paddr))
            return false;

        Port::readBlob(paddr, p + prevSize, gen.size());
        prevSize += gen.size();
    }

    return true;
}

void
TranslatingPort::readBlob(Addr addr, uint8_t *p, int size)
{
    if (!tryReadBlob(addr, p, size))
        fatal("readBlob(0x%x, ...) failed", addr);
}


bool
TranslatingPort::tryWriteBlob(Addr addr, uint8_t *p, int size)
{

    Addr paddr;
    int prevSize = 0;

    for (ChunkGenerator gen(addr, size, VMPageSize); !gen.done(); gen.next()) {

        if (!pTable->translate(gen.addr(), paddr)) {
            if (allocating == Always) {
                pTable->allocate(roundDown(gen.addr(), VMPageSize),
                                 VMPageSize);
            } else if (allocating == NextPage) {
                // check if we've accessed the next page on the stack
                if (!process->checkAndAllocNextPage(gen.addr()))
                    panic("Page table fault when accessing virtual address %#x "
                            "during functional write\n", gen.addr());
            } else {
                return false;
            }
            pTable->translate(gen.addr(), paddr);
        }

        Port::writeBlob(paddr, p + prevSize, gen.size());
        prevSize += gen.size();
    }

    return true;
}


void
TranslatingPort::writeBlob(Addr addr, uint8_t *p, int size)
{
    if (!tryWriteBlob(addr, p, size))
        fatal("writeBlob(0x%x, ...) failed", addr);
}

bool
TranslatingPort::tryMemsetBlob(Addr addr, uint8_t val, int size)
{
    Addr paddr;

    for (ChunkGenerator gen(addr, size, VMPageSize); !gen.done(); gen.next()) {

        if (!pTable->translate(gen.addr(), paddr)) {
            if (allocating == Always) {
                pTable->allocate(roundDown(gen.addr(), VMPageSize),
                                 VMPageSize);
                pTable->translate(gen.addr(), paddr);
            } else {
                return false;
            }
        }

        Port::memsetBlob(paddr, val, gen.size());
    }

    return true;
}

void
TranslatingPort::memsetBlob(Addr addr, uint8_t val, int size)
{
    if (!tryMemsetBlob(addr, val, size))
        fatal("memsetBlob(0x%x, ...) failed", addr);
}


bool
TranslatingPort::tryWriteString(Addr addr, const char *str)
{
    Addr paddr,vaddr;
    uint8_t c;

    vaddr = addr;

    do {
        c = *str++;
        if (!pTable->translate(vaddr++,paddr))
            return false;

        Port::writeBlob(paddr, &c, 1);
    } while (c);

    return true;
}

void
TranslatingPort::writeString(Addr addr, const char *str)
{
    if (!tryWriteString(addr, str))
        fatal("writeString(0x%x, ...) failed", addr);
}

bool
TranslatingPort::tryReadString(std::string &str, Addr addr)
{
    Addr paddr,vaddr;
    uint8_t c;

    vaddr = addr;

    do {
        if (!pTable->translate(vaddr++,paddr))
            return false;

        Port::readBlob(paddr, &c, 1);
        str += c;
    } while (c);

    return true;
}

void
TranslatingPort::readString(std::string &str, Addr addr)
{
    if (!tryReadString(str, addr))
        fatal("readString(0x%x, ...) failed", addr);
}

