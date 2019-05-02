/*
 * Copyright (c) 2011 ARM Limited
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
 *          Andreas Hansson
 */

#include "mem/se_translating_port_proxy.hh"

#include <string>

#include "arch/isa_traits.hh"
#include "base/chunk_generator.hh"
#include "config/the_isa.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/system.hh"

using namespace TheISA;

SETranslatingPortProxy::SETranslatingPortProxy(MasterPort& port, Process *p,
                                           AllocType alloc)
    : PortProxy(port, p->system->cacheLineSize()), pTable(p->pTable),
      process(p), allocating(alloc)
{ }

bool
SETranslatingPortProxy::tryReadBlob(Addr addr, void *p, int size) const
{
    int prevSize = 0;
    auto *bytes = static_cast<uint8_t *>(p);

    for (ChunkGenerator gen(addr, size, PageBytes); !gen.done(); gen.next()) {
        Addr paddr;

        if (!pTable->translate(gen.addr(),paddr))
            return false;

        PortProxy::readBlobPhys(paddr, 0, bytes + prevSize, gen.size());
        prevSize += gen.size();
    }

    return true;
}


bool
SETranslatingPortProxy::tryWriteBlob(Addr addr, const void *p, int size) const
{
    int prevSize = 0;
    auto *bytes = static_cast<const uint8_t *>(p);

    for (ChunkGenerator gen(addr, size, PageBytes); !gen.done(); gen.next()) {
        Addr paddr;

        if (!pTable->translate(gen.addr(), paddr)) {
            if (allocating == Always) {
                process->allocateMem(roundDown(gen.addr(), PageBytes),
                                     PageBytes);
            } else if (allocating == NextPage) {
                // check if we've accessed the next page on the stack
                if (!process->fixupStackFault(gen.addr()))
                    panic("Page table fault when accessing virtual address %#x "
                            "during functional write\n", gen.addr());
            } else {
                return false;
            }
            pTable->translate(gen.addr(), paddr);
        }

        PortProxy::writeBlobPhys(paddr, 0, bytes + prevSize, gen.size());
        prevSize += gen.size();
    }

    return true;
}


bool
SETranslatingPortProxy::tryMemsetBlob(Addr addr, uint8_t val, int size) const
{
    for (ChunkGenerator gen(addr, size, PageBytes); !gen.done(); gen.next()) {
        Addr paddr;

        if (!pTable->translate(gen.addr(), paddr)) {
            if (allocating == Always) {
                process->allocateMem(roundDown(gen.addr(), PageBytes),
                                     PageBytes);
                pTable->translate(gen.addr(), paddr);
            } else {
                return false;
            }
        }

        PortProxy::memsetBlobPhys(paddr, 0, val, gen.size());
    }

    return true;
}
