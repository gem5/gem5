/*
 * Copyright (c) 2015 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#ifndef __BASE_CIRCLEBUF_HH__
#define __BASE_CIRCLEBUF_HH__

#include <algorithm>
#include <cassert>
#include <vector>

#include "base/logging.hh"
#include "sim/serialize.hh"

/**
 * Circular buffer backed by a vector
 *
 * The data in the cricular buffer is stored in a standard
 * vector. _start designates the first element in the buffer and _stop
 * points to the last element + 1 (i.e., the position of the next
 * insertion). The _stop index may be outside the range of the backing
 * store, which means that the actual index must be calculated as
 * _stop % capacity.
 *
 * Invariants:
 * <ul>
 *   <li>_start <= _stop
 *   <li>_start < capacity
 *   <li>_stop < 2 * capacity
 * </ul>
 */
template<typename T>
class CircleBuf
{
  public:
    typedef T value_type;

  public:
    explicit CircleBuf(size_t size)
        : buf(size), _start(0), _stop(0) {}

    /** Is the buffer empty? */
    bool empty() const { return _stop == _start; }
    /**
     * Return the maximum number of elements that can be stored in
     * the buffer at any one time.
     */
    size_t capacity() const { return buf.size(); }
    /** Return the number of elements stored in the buffer. */
    size_t size() const { return _stop - _start; }

    /**
     * Remove all the elements in the buffer.
     *
     * Note: This does not actually remove elements from the backing
     * store.
     */
    void flush() {
        _start = 0;
        _stop = 0;
    }

    /**
     * Copy buffer contents without advancing the read pointer
     *
     * @param out Output iterator/pointer
     * @param len Number of elements to copy
     */
    template <class OutputIterator>
    void peek(OutputIterator out, size_t len) const {
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
    void peek(OutputIterator out, off_t offset, size_t len) const {
        panic_if(offset + len > size(),
                 "Trying to read past end of circular buffer.\n");

        const off_t real_start((offset + _start) % buf.size());
        if (real_start + len <= buf.size()) {
            std::copy(buf.begin() + real_start,
                      buf.begin() + real_start + len,
                      out);
        } else {
            const size_t head_size(buf.size() - real_start);
            const size_t tail_size(len - head_size);
            std::copy(buf.begin() + real_start, buf.end(),
                      out);
            std::copy(buf.begin(), buf.begin() + tail_size,
                      out + head_size);
        }
    }

    /**
     * Copy buffer contents and advance the read pointer
     *
     * @param out Output iterator/pointer
     * @param len Number of elements to read
     */
    template <class OutputIterator>
    void read(OutputIterator out, size_t len) {
        peek(out, len);

        _start += len;
        normalize();
    }

    /**
     * Add elements to the end of the ring buffers and advance.
     *
     * @param in Input iterator/pointer
     * @param len Number of elements to read
     */
    template <class InputIterator>
    void write(InputIterator in, size_t len) {
        // Writes that are larger than the backing store are allowed,
        // but only the last part of the buffer will be written.
        if (len > buf.size()) {
            in += len - buf.size();
            len = buf.size();
        }

        const size_t next(_stop % buf.size());
        const size_t head_len(std::min(buf.size() - next, len));

        std::copy(in, in + head_len, buf.begin() + next);
        std::copy(in + head_len, in + len, buf.begin());

        _stop += len;
        // We may have written past the old _start pointer. Readjust
        // the _start pointer to remove the oldest entries in that
        // case.
        if (size() > buf.size())
            _start = _stop - buf.size();

        normalize();
    }

  protected:
    /**
     * Normalize the start and stop pointers to ensure that pointer
     * invariants hold after updates.
     */
    void normalize() {
        if (_start >= buf.size()) {
            _stop -= buf.size();
            _start -= buf.size();
        }

        assert(_start < buf.size());
        assert(_stop < 2 * buf.size());
        assert(_start <= _stop);
    }

  protected:
    std::vector<value_type> buf;
    size_t _start;
    size_t _stop;

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
template<typename T>
class Fifo
{
  public:
    typedef T value_type;

  public:
    Fifo(size_t size)
        : buf(size) {}

    bool empty() const { return buf.empty(); }
    size_t size() const { return buf.size(); }
    size_t capacity() const { return buf.capacity(); }

    void flush() { buf.flush(); }

    template <class OutputIterator>
    void peek(OutputIterator out, size_t len) const { buf.peek(out, len); }
    template <class OutputIterator>
    void read(OutputIterator out, size_t len) { buf.read(out, len); }

    template <class InputIterator>
    void write(InputIterator in, size_t len) {
        panic_if(size() + len > capacity(),
                 "Trying to overfill FIFO buffer.\n");
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
arrayParamIn(CheckpointIn &cp, const std::string &name,
             CircleBuf<T> &param)
{
    std::vector<T> temp;
    arrayParamIn(cp, name, temp);

    param.flush();
    param.write(temp.cbegin(), temp.size());
}

template <typename T>
void
arrayParamOut(CheckpointOut &cp, const std::string &name,
              const Fifo<T> &param)
{
    std::vector<T> temp(param.size());
    param.peek(temp.begin(), temp.size());
    arrayParamOut(cp, name, temp);
}

template <typename T>
void
arrayParamIn(CheckpointIn &cp, const std::string &name,
             Fifo<T> &param)
{
    std::vector<T> temp;
    arrayParamIn(cp, name, temp);

    fatal_if(param.capacity() < temp.size(),
             "Trying to unserialize data into too small FIFO\n");

    param.flush();
    param.write(temp.cbegin(), temp.size());
}

#endif // __BASE_CIRCLEBUF_HH__
