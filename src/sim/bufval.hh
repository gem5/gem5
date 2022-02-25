/*
 * Copyright 2022 Google Inc.
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

#ifndef __SIM_BUFVAL_HH__
#define __SIM_BUFVAL_HH__

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include "enums/ByteOrder.hh"

namespace gem5
{

// Extract an integer with a given endianness from a variably sized buffer.
// Returns the value extraced (if any) and a bool indicating success.
std::pair<std::uint64_t, bool> getUintX(const void *buf, std::size_t bytes,
        ByteOrder endian);

// Set a variably sized buffer to an integer value with a given endianness.
// Returns whether the assignment was successful.
bool setUintX(std::uint64_t val, void *buf, std::size_t bytes,
        ByteOrder endian);

// Print an integer with a given endianness into a string from a variably
// sized buffer. Returns the string (if any) and a bool indicating success.
std::pair<std::string, bool> printUintX(const void *buf, std::size_t bytes,
        ByteOrder endian);

// Print a buffer as "chunk_size" sized groups of bytes. The endianness
// determines if the bytes are output in memory order (little) or inverse of
// memory order (big). The default is in memory order so that this acts like
// a hexdump type utility. The return value is a string holding the printed
// bytes.
std::string printByteBuf(const void *buf, std::size_t bytes,
        ByteOrder endian=ByteOrder::little, std::size_t chunk_size=4);

} // namespace gem5

#endif // __SIM_BUFVAL_HH__
