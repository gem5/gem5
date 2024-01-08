/*
 * Copyright (c) 2019-2020 Inria
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

/** @file
 * Implementation of the specialized sub-compressors used by BDI. @see BDI
 */

#include "base/trace.hh"
#include "mem/cache/compressors/base_delta_impl.hh"
#include "params/Base16Delta8.hh"
#include "params/Base32Delta16.hh"
#include "params/Base32Delta8.hh"
#include "params/Base64Delta16.hh"
#include "params/Base64Delta32.hh"
#include "params/Base64Delta8.hh"

namespace gem5
{

namespace compression
{

Base64Delta8::Base64Delta8(const Params &p)
    : BaseDelta<uint64_t, 8>(p)
{
}

Base64Delta16::Base64Delta16(const Params &p)
    : BaseDelta<uint64_t, 16>(p)
{
}

Base64Delta32::Base64Delta32(const Params &p)
    : BaseDelta<uint64_t, 32>(p)
{
}

Base32Delta8::Base32Delta8(const Params &p)
    : BaseDelta<uint32_t, 8>(p)
{
}

Base32Delta16::Base32Delta16(const Params &p)
    : BaseDelta<uint32_t, 16>(p)
{
}

Base16Delta8::Base16Delta8(const Params &p)
    : BaseDelta<uint16_t, 8>(p)
{
}

} // namespace compression
} // namespace gem5
