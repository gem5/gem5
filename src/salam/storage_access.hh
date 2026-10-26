/*
 * Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
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

#ifndef __SALAM_STORAGE_ACCESS_HH__
#define __SALAM_STORAGE_ACCESS_HH__

#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

#include "mem/packet.hh"

namespace SALAM
{

struct StorageAccess
{
    uint64_t words = 0;
    uint64_t bits = 0;
    uint64_t partialWords = 0;
};

// Count touched physical words, including partial and unaligned accesses.
// Offsets are relative to the physical memory, not the requestor.
inline StorageAccess
storageAccess(uint64_t offset, unsigned size, unsigned word_bytes,
              const std::vector<bool> &enabled = {})
{
    if (!word_bytes || (!enabled.empty() && enabled.size() != size)) {
        throw std::invalid_argument("invalid storage access geometry");
    }
    std::map<uint64_t, unsigned> touched;
    for (unsigned i = 0; i < size; ++i) {
        if (enabled.empty() || enabled[i]) {
            ++touched[(offset + i) / word_bytes];
        }
    }
    StorageAccess result;
    result.words = touched.size();
    for (const auto &entry : touched) {
        result.bits += 8 * entry.second;
        result.partialWords += entry.second != word_bytes;
    }
    return result;
}

// A serviced all-disabled write changes no storage cells. The caller still
// performs its existing packet response, byte counters and deferred work.
inline bool
hasStorageTransfer(const gem5::Packet &packet)
{
    if (!packet.getSize()) {
        return false;
    }
    if (!packet.isWrite()) {
        return true;
    }
    const auto &enabled = packet.req->getByteEnable();
    return enabled.empty() || std::any_of(enabled.begin(), enabled.end(),
                                          [](bool bit) { return bit; });
}

inline StorageAccess
storageAccess(const gem5::Packet &packet, uint64_t memory_start,
              unsigned word_bytes)
{
    return storageAccess(
        packet.getAddr() - memory_start, packet.getSize(), word_bytes,
        packet.isWrite() ? packet.req->getByteEnable() : std::vector<bool>{});
}

} // namespace SALAM

#endif // __SALAM_STORAGE_ACCESS_HH__
