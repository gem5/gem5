/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/amdgpu/vega/pagetable.hh"

#include "sim/serialize.hh"

namespace gem5
{
namespace VegaISA
{

/**
 * Serialize all of the POD fields in a TLB entry. These fields contain all
 * of the information needed to handle any of the function calls in the entry.
 */
void
VegaTlbEntry::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(paddr);
    SERIALIZE_SCALAR(vaddr);
    SERIALIZE_SCALAR(logBytes);
    SERIALIZE_SCALAR(vmid);
    SERIALIZE_SCALAR(pte);
    SERIALIZE_SCALAR(lruSeq);
}

/**
 * Unserialize all of the POD fields in a TLB entry. These fields contain all
 * of the information needed to handle any of the function calls in the entry.
 */
void
VegaTlbEntry::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(paddr);
    UNSERIALIZE_SCALAR(vaddr);
    UNSERIALIZE_SCALAR(logBytes);
    UNSERIALIZE_SCALAR(vmid);
    UNSERIALIZE_SCALAR(pte);
    UNSERIALIZE_SCALAR(lruSeq);
}

} // namespace VegaISA
} // namespace gem5
