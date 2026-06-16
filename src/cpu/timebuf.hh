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

#ifndef __BASE_TIMEBUF_HH__
#define __BASE_TIMEBUF_HH__

#include <cassert>
#include <vector>

namespace gem5
{

template <class T>
class TimeBuffer
{
  protected:
    int past;
    int future;
    unsigned size;
    int _id;

    std::vector<T> data;
    unsigned base;

    void valid(int idx) const
    {
        assert (idx >= -past && idx <= future);
    }

  public:
    friend class wire;
    class wire
    {
        friend class TimeBuffer;
      protected:
        TimeBuffer<T> *buffer;
        int index;

        void set(int idx)
        {
            buffer->valid(idx);
            index = idx;
        }

        wire(TimeBuffer<T> *buf, int i)
            : buffer(buf), index(i)
        { }

      public:
        wire()
        { }

        wire(const wire &i)
            : buffer(i.buffer), index(i.index)
        { }

        const wire &operator=(const wire &i)
        {
            buffer = i.buffer;
            set(i.index);
            return *this;
        }

        const wire &operator=(int idx)
        {
            set(idx);
            return *this;
        }

        const wire &operator+=(int offset)
        {
            set(index + offset);
            return *this;
        }

        const wire &operator-=(int offset)
        {
            set(index - offset);
            return *this;
        }

        wire &operator++()
        {
            set(index + 1);
            return *this;
        }

        wire &operator++(int)
        {
            int i = index;
            set(index + 1);
            return wire(this, i);
        }

        wire &operator--()
        {
            set(index - 1);
            return *this;
        }

        wire &operator--(int)
        {
            int i = index;
            set(index - 1);
            return wire(this, i);
        }
        T &operator*() const { return *buffer->access(index); }
        T *operator->() const { return buffer->access(index); }
    };


  public:
    TimeBuffer(int p, int f)
        : past(p), future(f), size(past + future + 1),
          data(size), base(0)
    {
        assert(past >= 0 && future >= 0);
        _id = -1;
    }

    TimeBuffer()
        : past(0), future(0), size(0), data(0), base(0)
    {
    }

    void id(int id)
    {
        _id = id;
    }

    int id()
    {
        return _id;
    }

    void
    advance()
    {
        if (++base >= size)
            base = 0;

        int ptr = base + future;
        if (ptr >= (int)size)
            ptr -= size;
        data[ptr] = T{};
    }

  protected:
    //Calculate the index into this->index for element at position idx
    //relative to now
    inline int calculateVectorIndex(int idx) const
    {
        //Need more complex math here to calculate index.
        valid(idx);

        int vector_index = idx + base;
        if (vector_index >= (int)size) {
            vector_index -= size;
        } else if (vector_index < 0) {
            vector_index += size;
        }

        return vector_index;
    }

  public:
    T *access(int idx)
    {
        int vector_index = calculateVectorIndex(idx);

        return &data[vector_index];
    }

    T &operator[](int idx)
    {
        int vector_index = calculateVectorIndex(idx);

        return data[vector_index];
    }

    const T &operator[] (int idx) const
    {
        int vector_index = calculateVectorIndex(idx);

        return data[vector_index];
    }

    wire getWire(int idx)
    {
        valid(idx);

        return wire(this, idx);
    }

    wire zero()
    {
        return wire(this, 0);
    }

    unsigned getSize()
    {
        return size;
    }

    int
    getPast() const
    {
        return past;
    }

    int
    getFuture() const
    {
        return future;
    }
};

} // namespace gem5

#endif // __BASE_TIMEBUF_HH__
