/*
 * Copyright (c) 2015,2017-2018 ARM Limited
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

#ifndef __BASE_CIRCLEBUF_HH__
#define __BASE_CIRCLEBUF_HH__

#include <algorithm>
#include <cassert>
#include <iterator>
#include <vector>

#include "base/logging.hh"
#include "sim/serialize.hh"

namespace gem5
{

/**
 * Circular buffer backed by a vector.
 *
 * The data in the cricular buffer is stored in a standard vector.
 */
template <typename T>
class CircleBuf
{
  private:
    std::vector<T> buffer;
    size_t start = 0;
    size_t used = 0;
    size_t maxSize;

  public:
    using value_type = T;

    explicit CircleBuf(size_t size) : buffer(size), maxSize(size) {}

    bool
    empty() const
    {
        return used == 0;
    }

    size_t
    size() const
    {
        return used;
    }

    size_t
    capacity() const
    {
        return maxSize;
    }

    /**
     * Throw away any data in the buffer.
     */
    void
    flush()
    {
        start = 0;
        used = 0;
    }

    /**
     * Copy buffer contents without advancing the read pointer
     *
     * @param out Output iterator/pointer
     * @param len Number of elements to copy
     */
    template <class OutputIterator>
    void
    peek(OutputIterator out, size_t len) const
    {
        peek(out, 0, len);
    }

    /**
     * Copy buffer contents without advancing the read pointer
     *
     * @param out Output iterator/pointer
     * @param offset Offset into the ring buffer
     * @param len Number of elements to copy
     */
    template <class OutputIterator>
    void
    peek(OutputIterator out, off_t offset, size_t len) const
    {
        panic_if(offset + len > used,
                 "Trying to read past end of circular buffer.");

        if (!len)
            return;

        // The iterator for the next byte to copy out.
        auto next_it = buffer.begin() + (start + offset) % maxSize;
        // How much there is to copy from until the end of the buffer.
        const size_t to_end = buffer.end() - next_it;

        // If the data to be copied wraps, take care of the first part.
        if (to_end < len) {
            // Copy it.
            out = std::copy_n(next_it, to_end, out);
            // Start copying again at the start of buffer.
            next_it = buffer.begin();
            len -= to_end;
        }
        // Copy the remaining (or only) chunk of data.
        std::copy_n(next_it, len, out);
    }

    /**
     * Copy buffer contents and advance the read pointer
     *
     * @param out Output iterator/pointer
     * @param len Number of elements to read
     */
    template <class OutputIterator>
    void
    read(OutputIterator out, size_t len)
    {
        peek(out, len);
        used -= len;
        start += len;
    }

    /**
     * Add elements to the end of the ring buffers and advance. Writes which
     * would exceed the capacity of the queue fill the avaialble space, and
     * then continue overwriting the head of the queue. The head advances as
     * if that data had been read out.
     *
     * @param in Input iterator/pointer
     * @param len Number of elements to read
     */
    template <class InputIterator>
    void
    write(InputIterator in, size_t len)
    {
        if (!len)
            return;

        // Writes that are larger than the buffer size are allowed, but only
        // the last part of the date will be written since the rest will be
        // overwritten and not remain in the buffer.
        if (len > maxSize) {
            in += len - maxSize;
            flush();
            len = maxSize;
        }

        // How much existing data will be overwritten?
        const size_t total_bytes = used + len;
        const size_t overflow =
            total_bytes > maxSize ? total_bytes - maxSize : 0;
        // The iterator of the next byte to add.
        auto next_it = buffer.begin() + (start + used) % maxSize;
        // How much there is to copy to the end of the buffer.
        const size_t to_end = buffer.end() - next_it;

        // If this addition wraps, take care of the first part here.
        if (to_end < len) {
            // Copy it.
            std::copy_n(in, to_end, next_it);
            // Update state to reflect what's left.
            next_it = buffer.begin();
            std::advance(in, to_end);
            len -= to_end;
            used += to_end;
        }
        // Copy the remaining (or only) chunk of data.
        std::copy_n(in, len, next_it);
        used += len;

        // Don't count data that was overwritten.
        used -= overflow;
        start += overflow;
    }
};

/**
 * Simple FIFO implementation backed by a circular buffer.
 *
 * This class provides the same basic functionallity as the circular
 * buffer with the folling differences:
 * <ul>
 *    <li>Writes are checked to ensure that overflows can't happen.
 *    <li>Unserialization ensures that the data in the checkpoint fits
 *        in the buffer.
 * </ul>
 */
template <typename T>
class Fifo
{
  public:
    typedef T value_type;

  public:
    Fifo(size_t size) : buf(size) {}

    bool
    empty() const
    {
        return buf.empty();
    }

    size_t
    size() const
    {
        return buf.size();
    }

    size_t
    capacity() const
    {
        return buf.capacity();
    }

    void
    flush()
    {
        buf.flush();
    }

    template <class OutputIterator>
    void
    peek(OutputIterator out, size_t len) const
    {
        buf.peek(out, len);
    }

    template <class OutputIterator>
    void
    read(OutputIterator out, size_t len)
    {
        buf.read(out, len);
    }

    template <class InputIterator>
    void
    write(InputIterator in, size_t len)
    {
        panic_if(size() + len > capacity(), "Trying to overfill FIFO buffer.");
        buf.write(in, len);
    }

  private:
    CircleBuf<value_type> buf;
};

template <typename T>
void
arrayParamOut(CheckpointOut &cp, const std::string &name,
              const CircleBuf<T> &param)
{
    std::vector<T> temp(param.size());
    param.peek(temp.begin(), temp.size());
    arrayParamOut(cp, name, temp);
}

template <typename T>
void
arrayParamIn(CheckpointIn &cp, const std::string &name, CircleBuf<T> &param)
{
    std::vector<T> temp;
    arrayParamIn(cp, name, temp);

    param.flush();
    param.write(temp.cbegin(), temp.size());
}

template <typename T>
void
arrayParamOut(CheckpointOut &cp, const std::string &name, const Fifo<T> &param)
{
    std::vector<T> temp(param.size());
    param.peek(temp.begin(), temp.size());
    arrayParamOut(cp, name, temp);
}

template <typename T>
void
arrayParamIn(CheckpointIn &cp, const std::string &name, Fifo<T> &param)
{
    std::vector<T> temp;
    arrayParamIn(cp, name, temp);

    fatal_if(param.capacity() < temp.size(),
             "Trying to unserialize data into too small FIFO");

    param.flush();
    param.write(temp.cbegin(), temp.size());
}

} // namespace gem5

#endif // __BASE_CIRCLEBUF_HH__
