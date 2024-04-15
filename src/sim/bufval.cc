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

#include "sim/bufval.hh"

#include <cassert>
#include <sstream>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "sim/byteswap.hh"

namespace gem5
{

std::pair<std::uint64_t, bool>
getUintX(const void *buf, std::size_t bytes, ByteOrder endian)
{
    assert(buf);
    switch (bytes) {
    case sizeof(std::uint64_t):
        return { gtoh(*(const std::uint64_t *)buf, endian), true };
    case sizeof(std::uint32_t):
        return { gtoh(*(const std::uint32_t *)buf, endian), true };
    case sizeof(std::uint16_t):
        return { gtoh(*(const std::uint16_t *)buf, endian), true };
    case sizeof(std::uint8_t):
        return { gtoh(*(const std::uint8_t *)buf, endian), true };
    default:
        return { 0, false };
    }
}

bool
setUintX(std::uint64_t val, void *buf, std::size_t bytes, ByteOrder endian)
{
    assert(buf);

    switch (bytes) {
    case sizeof(std::uint64_t):
        *(std::uint64_t *)buf = htog<std::uint64_t>(val, endian);
        return true;
    case sizeof(std::uint32_t):
        *(std::uint32_t *)buf = htog<std::uint32_t>(val, endian);
        return true;
    case sizeof(std::uint16_t):
        *(std::uint16_t *)buf = htog<std::uint16_t>(val, endian);
        return true;
    case sizeof(std::uint8_t):
        *(std::uint8_t *)buf = htog<std::uint8_t>(val, endian);
        return true;
    default:
        return false;
    }
}

std::pair<std::string, bool>
printUintX(const void *buf, std::size_t bytes, ByteOrder endian)
{
    auto [val, success] = getUintX(buf, bytes, endian);
    if (!success)
        return { "", false };

    std::ostringstream out;
    out << "0x";
    out.width(2 * bytes);
    out.fill('0');
    out.setf(std::ios::hex, std::ios::basefield);
    out << val;

    return { out.str(), true };
}

std::string
printByteBuf(const void *buf, std::size_t bytes, ByteOrder endian,
             std::size_t chunk_size)
{
    assert(buf);

    std::ostringstream out;
    out << "[";

    out.width(2);
    out.fill('0');
    out.setf(std::ios::hex, std::ios::basefield);

    // Bytes that fall outside of a complete chunk. Will always be MSBs.
    size_t extra = bytes % chunk_size;

    const uint8_t *ptr = (const uint8_t *)buf;
    int step = 1;

    if (endian == ByteOrder::big) {
        step = -1;
        ptr = ptr + bytes - 1;

        // If there's an incomplete chunk, start with that.
        if (extra) {
            bytes -= extra;
            while (extra--) {
                out.width(2);
                out << (unsigned)*ptr;
                ptr += step;
            }
            if (bytes)
                out << " ";
        }
    }

    // Print all the complete chunks.
    while (bytes >= chunk_size) {
        for (int i = 0; i < chunk_size; i++) {
            out.width(2);
            out << (unsigned)*ptr;
            ptr += step;
        }
        bytes -= chunk_size;
        if (bytes)
            out << " ";
    }

    // Print any trailing leftovers. Only happens for little endian.
    while (bytes--) {
        out.width(2);
        out << (unsigned)*ptr;
        ptr += step;
    }

    out.width(0);
    out << "]";

    return out.str();
}

} // namespace gem5
