/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2020 Barkhausen Institut
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

#include "arch/riscv/pagetable.hh"

#include "sim/serialize.hh"

namespace gem5
{

namespace RiscvISA
{

void
TlbEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(paddr);
    SERIALIZE_SCALAR(vaddr);
    SERIALIZE_SCALAR(logBytes);
    SERIALIZE_SCALAR(asid);
    SERIALIZE_SCALAR(pte);
    SERIALIZE_SCALAR(lruSeq);
}

void
TlbEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(paddr);
    UNSERIALIZE_SCALAR(vaddr);
    UNSERIALIZE_SCALAR(logBytes);
    UNSERIALIZE_SCALAR(asid);
    UNSERIALIZE_SCALAR(pte);
    UNSERIALIZE_SCALAR(lruSeq);
}

Addr
getVPNFromVAddr(Addr vaddr, Addr mode)
{
    switch (mode) {
    case BARE:
        return vaddr >> 12;
    case SV39:
        return bits(vaddr, 38, 12);
    case SV48:
        return bits(vaddr, 47, 12);
    case SV57:
        return bits(vaddr, 56, 12);
    default:
        panic("Unknown address translation mode %d\n", mode);
    }
}

} // namespace RiscvISA
} // namespace gem5
