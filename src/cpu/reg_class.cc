/*
 * Copyright (c) 2016-2017 ARM Limited
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
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
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

#include "cpu/reg_class.hh"

#include <sstream>

#include "base/cprintf.hh"

namespace gem5
{

std::string
RegClassOps::regName(const RegId &id) const
{
    return csprintf("r%d", id.index());
}

std::string
RegClassOps::valString(const void *val, size_t size) const
{
    // If this is just a RegVal, or could be interpreted as one, print it
    // that way.
    if (size == sizeof(uint64_t))
        return csprintf("0x%016x", *(const uint64_t *)val);
    else if (size == sizeof(uint32_t))
        return csprintf("0x%08x", *(const uint32_t *)val);
    else if (size == sizeof(uint16_t))
        return csprintf("0x%04x", *(const uint16_t *)val);
    else if (size == sizeof(uint8_t))
        return csprintf("0x%02x", *(const uint8_t *)val);

    // Otherwise, print it as a sequence of bytes, 4 in a chunk, separated by
    // spaces, and all surrounded by []s.

    std::stringstream out;
    ccprintf(out, "[");

    constexpr size_t chunk_size = 4;
    const uint8_t *bytes = (const uint8_t *)val;

    while (size >= chunk_size) {
        size -= chunk_size;
        if (size) {
            ccprintf(out, "%02x%02x%02x%02x ", bytes[0], bytes[1], bytes[2],
                    bytes[3]);
        } else {
            ccprintf(out, "%02x%02x%02x%02x", bytes[0], bytes[1], bytes[2],
                    bytes[3]);
        }
        bytes += chunk_size;
    }

    while (size--)
        ccprintf(out, "%02x", *bytes++);

    ccprintf(out, "]");

    return out.str();
}

const char *RegId::regClassStrings[] = {
    "IntRegClass",
    "FloatRegClass",
    "VecRegClass",
    "VecElemClass",
    "VecPredRegClass",
    "CCRegClass",
    "MiscRegClass"
};

} // namespace gem5
