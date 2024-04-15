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

#ifndef __DEV_NET_PKTFIFO_HH__
#define __DEV_NET_PKTFIFO_HH__

#include <iosfwd>
#include <list>
#include <string>

#include "base/logging.hh"
#include "dev/net/etherpkt.hh"
#include "sim/serialize.hh"

namespace gem5
{

class Checkpoint;

struct PacketFifoEntry
{
    EthPacketPtr packet;
    uint64_t number;
    unsigned slack;
    int priv;

    PacketFifoEntry() { clear(); }

    PacketFifoEntry(const PacketFifoEntry &s)
        : packet(s.packet), number(s.number), slack(s.slack), priv(s.priv)
    {}

    PacketFifoEntry(EthPacketPtr p, uint64_t n)
        : packet(p), number(n), slack(0), priv(-1)
    {}

    void
    clear()
    {
        packet = NULL;
        number = 0;
        slack = 0;
        priv = -1;
    }

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);
};

class PacketFifo
{
  public:
    typedef std::list<PacketFifoEntry> fifo_list;
    typedef fifo_list::iterator iterator;
    typedef fifo_list::const_iterator const_iterator;

  protected:
    std::list<PacketFifoEntry> fifo;
    uint64_t _counter;
    unsigned _maxsize;
    unsigned _size;
    unsigned _reserved;

  public:
    explicit PacketFifo(int max)
        : _counter(0), _maxsize(max), _size(0), _reserved(0)
    {}

    virtual ~PacketFifo() {}

    unsigned
    packets() const
    {
        return fifo.size();
    }

    unsigned
    maxsize() const
    {
        return _maxsize;
    }

    unsigned
    size() const
    {
        return _size;
    }

    unsigned
    reserved() const
    {
        return _reserved;
    }

    unsigned
    avail() const
    {
        return _maxsize - _size - _reserved;
    }

    bool
    empty() const
    {
        return size() <= 0;
    }

    bool
    full() const
    {
        return avail() <= 0;
    }

    unsigned
    reserve(unsigned len = 0)
    {
        assert(avail() >= len);
        _reserved += len;
        return _reserved;
    }

    iterator
    begin()
    {
        return fifo.begin();
    }

    iterator
    end()
    {
        return fifo.end();
    }

    const_iterator
    begin() const
    {
        return fifo.begin();
    }

    const_iterator
    end() const
    {
        return fifo.end();
    }

    EthPacketPtr
    front()
    {
        return fifo.begin()->packet;
    }

    bool
    push(EthPacketPtr ptr)
    {
        assert(ptr->length);
        assert(_reserved <= ptr->length);
        if (avail() < ptr->length - _reserved)
            return false;

        _size += ptr->length;

        PacketFifoEntry entry;
        entry.packet = ptr;
        entry.number = _counter++;
        fifo.push_back(entry);
        _reserved = 0;
        return true;
    }

    void
    pop()
    {
        if (empty())
            return;

        iterator entry = fifo.begin();
        _size -= entry->packet->length;
        _size -= entry->slack;
        entry->packet = NULL;
        fifo.pop_front();
    }

    void
    clear()
    {
        for (iterator i = begin(); i != end(); ++i)
            i->clear();
        fifo.clear();
        _size = 0;
        _reserved = 0;
    }

    void
    remove(iterator i)
    {
        if (i != fifo.begin()) {
            iterator prev = i;
            --prev;
            assert(prev != fifo.end());
            prev->slack += i->packet->length;
            prev->slack += i->slack;
        } else {
            _size -= i->packet->length;
            _size -= i->slack;
        }

        i->clear();
        fifo.erase(i);
    }

    bool copyout(void *dest, unsigned offset, unsigned len);

    int
    countPacketsBefore(const_iterator i) const
    {
        if (i == fifo.end())
            return 0;
        return i->number - fifo.begin()->number;
    }

    int
    countPacketsAfter(const_iterator i) const
    {
        auto end = fifo.end();
        if (i == end)
            return 0;
        return (--end)->number - i->number;
    }

    void
    check() const
    {
        unsigned total = 0;
        for (auto i = begin(); i != end(); ++i)
            total += i->packet->length + i->slack;

        if (total != _size)
            panic("total (%d) is not == to size (%d)\n", total, _size);
    }

    /**
     * Serialization stuff
     */
  public:
    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);
};

} // namespace gem5

#endif // __DEV_NET_PKTFIFO_HH__
