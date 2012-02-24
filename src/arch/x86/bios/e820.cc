/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
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
 * Authors: Gabe Black
 */

#include "arch/x86/bios/e820.hh"
#include "arch/x86/isa_traits.hh"
#include "mem/port_proxy.hh"
#include "sim/byteswap.hh"

using namespace std;
using namespace X86ISA;

template<class T>
void writeVal(T val, PortProxy& proxy, Addr &addr)
{
    T guestVal = htog(val);
    proxy.writeBlob(addr, (uint8_t *)&guestVal, sizeof(T));
    addr += sizeof(T);
}

void X86ISA::E820Table::writeTo(PortProxy& proxy, Addr countAddr, Addr addr)
{
    uint8_t e820Nr = entries.size();

    // Make sure the number of entries isn't bigger than what the kernel
    // would be capable of handling.
    assert(e820Nr <= 128);

    uint8_t guestE820Nr = htog(e820Nr);

    proxy.writeBlob(countAddr, (uint8_t *)&guestE820Nr, sizeof(guestE820Nr));

    for (int i = 0; i < e820Nr; i++) {
        writeVal(entries[i]->addr, proxy, addr);
        writeVal(entries[i]->size, proxy, addr);
        writeVal(entries[i]->type, proxy, addr);
    }
}

E820Table *
X86E820TableParams::create()
{
    return new E820Table(this);
}

E820Entry *
X86E820EntryParams::create()
{
    return new E820Entry(this);
}
