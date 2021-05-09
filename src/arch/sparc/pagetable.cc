/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "arch/sparc/pagetable.hh"

#include "sim/serialize.hh"

namespace gem5
{

namespace SparcISA
{

void
TlbEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(range.va);
    SERIALIZE_SCALAR(range.size);
    SERIALIZE_SCALAR(range.contextId);
    SERIALIZE_SCALAR(range.partitionId);
    SERIALIZE_SCALAR(range.real);
    uint64_t entry4u = 0;
    if (valid)
        entry4u = pte();
    SERIALIZE_SCALAR(entry4u);
    SERIALIZE_SCALAR(used);
    SERIALIZE_SCALAR(valid);
}


void
TlbEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(range.va);
    UNSERIALIZE_SCALAR(range.size);
    UNSERIALIZE_SCALAR(range.contextId);
    UNSERIALIZE_SCALAR(range.partitionId);
    UNSERIALIZE_SCALAR(range.real);
    uint64_t entry4u;
    UNSERIALIZE_SCALAR(entry4u);
    if (entry4u)
        pte.populate(entry4u);
    UNSERIALIZE_SCALAR(used);
    UNSERIALIZE_SCALAR(valid);
}


int PageTableEntry::pageSizes[] =
    { 8 * 1024, 64 * 1024, 0, 4 * 1024 * 1024, 0, 256 * 1024 * 1024L} ;


}

} // namespace gem5
