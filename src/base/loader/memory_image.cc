/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include "base/loader/memory_image.hh"
#include "mem/port_proxy.hh"

namespace gem5
{

namespace loader
{

bool
MemoryImage::writeSegment(const Segment &seg, const PortProxy &proxy) const
{
    if (seg.size != 0) {
        if (seg.data) {
            proxy.writeBlob(seg.base, seg.data, seg.size);
        } else {
            // no image: must be bss
            proxy.memsetBlob(seg.base, 0, seg.size);
        }
    }
    return true;
}

bool
MemoryImage::write(const PortProxy &proxy) const
{
    for (auto &seg : _segments)
        if (!writeSegment(seg, proxy))
            return false;
    return true;
}

MemoryImage &
MemoryImage::move(std::function<Addr(Addr)> mapper)
{
    for (auto &seg : _segments)
        seg.base = mapper(seg.base);
    return *this;
}

} // namespace loader
} // namespace gem5
