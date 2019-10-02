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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __BASE_LOADER_MEMORY_IMAGE_HH__
#define __BASE_LOADER_MEMORY_IMAGE_HH__

#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"

class PortProxy;
class Process;
class ProcessParams;
class SymbolTable;

class MemoryImage
{
  public:
    struct Segment
    {
        std::string name;
        Addr base;
        uint8_t *data;
        size_t size;
    };

    MemoryImage() {}

    MemoryImage(std::initializer_list<Segment> new_segs)
    {
        for (auto &seg: new_segs)
            addSegment(seg);
    }

  private:
    std::vector<Segment> _segments;
    bool writeSegment(const Segment &seg, const PortProxy &proxy) const;

  public:
    const std::vector<Segment> &
    segments() const
    {
        return _segments;
    }

    void
    addSegment(const Segment &seg)
    {
        _segments.emplace_back(seg);
    }

    void
    addSegment(std::string name, Addr base, uint8_t *data, size_t size)
    {
        _segments.push_back(Segment({name, base, data, size}));
    }

    bool write(const PortProxy &proxy) const;
    MemoryImage &move(std::function<Addr(Addr)> mapper);
    MemoryImage &
    offset(Addr by)
    {
        return move([by](Addr a){ return by + a; });
    }
    MemoryImage &
    mask(Addr m) {
        return move([m](Addr a) { return a & m; });
    }

    Addr
    maxAddr() const
    {
        Addr max = 0;
        for (auto &seg: _segments) {
            Addr end = seg.base + seg.size;
            if (end > max)
                max = end;
        }
        return max;
    }

    Addr
    minAddr() const
    {
        Addr min = MaxAddr;
        for (auto &seg: _segments)
            if (seg.base < min)
                min = seg.base;
        return min;
    }

    bool
    contains(Addr addr) const
    {
        for (auto &seg: _segments) {
            Addr start = seg.base;
            Addr end = seg.base + seg.size;
            if (addr >= start && addr < end)
                return true;
        }
        return false;
    }
};

static inline std::ostream &
operator << (std::ostream &os, const MemoryImage::Segment &seg)
{
    ccprintf(os, "%s: %#x %d", seg.name, seg.base, seg.size);
    return os;
}


#endif // __BASE_LOADER_MEMORY_IMAGE_HH__
