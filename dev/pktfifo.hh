/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __DEV_PKTFIFO_HH__
#define __DEV_PKTFIFO_HH__

#include <iosfwd>
#include <list>
#include <string>

#include "dev/etherpkt.hh"
#include "sim/serialize.hh"

class Checkpoint;
class PacketFifo
{
  protected:
    std::list<PacketPtr> fifo;
    int _maxsize;
    int _size;
    int _reserved;

  public:
    explicit PacketFifo(int max) : _maxsize(max), _size(0), _reserved(0) {}
    virtual ~PacketFifo() {}

    int packets() const { return fifo.size(); }
    int maxsize() const { return _maxsize; }
    int size() const { return _size; }
    int reserved() const { return _reserved; }
    int avail() const { return _maxsize - _size - _reserved; }
    bool empty() const { return size() <= 0; }
    bool full() const { return avail() <= 0; }

    int reserve(int len = 0)
    {
        _reserved += len;
        assert(avail() >= 0);
        return _reserved;
    }

    bool push(PacketPtr ptr)
    {
        assert(_reserved <= ptr->length);
        if (avail() < ptr->length - _reserved)
            return false;

        _size += ptr->length;
        fifo.push_back(ptr);
        _reserved = 0;
        return true;
    }

    PacketPtr front() { return fifo.front(); }

    void pop()
    {
        if (empty())
            return;

        _size -= fifo.front()->length;
        fifo.front() = NULL;
        fifo.pop_front();
    }

    void clear()
    {
        fifo.clear();
        _size = 0;
        _reserved = 0;
    }

/**
 * Serialization stuff
 */
  public:
    void serialize(const std::string &base, std::ostream &os);
    void unserialize(const std::string &base,
                     Checkpoint *cp, const std::string &section);
};

#endif // __DEV_PKTFIFO_HH__
