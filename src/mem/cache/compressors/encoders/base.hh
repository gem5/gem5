/*
 * Copyright (c) 2019, 2020 Inria
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

#ifndef __MEM_CACHE_COMPRESSORS_ENCODERS_BASE_HH__
#define __MEM_CACHE_COMPRESSORS_ENCODERS_BASE_HH__

#include <cstdint>

#include "base/compiler.hh"

namespace gem5
{

namespace compression
{
GEM5_DEPRECATED_NAMESPACE(Encoder, encoder);
namespace encoder
{

struct Code
{
    /** Only the LSB of the code are relevant. */
    uint64_t code;
    /** Number of bits in the code. */
    unsigned length;
};

/**
 * Base class for encoders. The goal of encoders is to provide an alternative
 * representation to values, ideally shorter than the value. The alternative
 * representation is called a code.
 */
class Base
{
  public:
    Base() {}
    virtual ~Base() = default;

    /**
     * The function responsible for the generation of the alternative value.
     * If the size of the returning Code is greater than the maximum undelying
     * type's size (e.g., 64 bits) the encoding results should be discarded.
     *
     * @param The value to be encoded.
     * @return The encoded value.
     */
    virtual Code encode(const uint64_t val) const = 0;

    /**
     * Decode a value.
     * @sa encode()
     *
     * @param code The encoded value.
     * @return The original value.
     */
    virtual uint64_t decode(const uint64_t code) const = 0;
};

} // namespace encoder
} // namespace compression
} // namespace gem5

#endif //__MEM_CACHE_COMPRESSORS_ENCODERS_BASE_HH__
