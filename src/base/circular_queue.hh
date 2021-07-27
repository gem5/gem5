/*
 * Copyright (c) 2017-2018 ARM Limited
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

#ifndef __BASE_CIRCULAR_QUEUE_HH__
#define __BASE_CIRCULAR_QUEUE_HH__

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <type_traits>
#include <vector>

namespace gem5
{

/** Circular queue.
 * Circular queue implemented in a standard vector. All indices are
 * monotonically increasing, and modulo is used at access time to alias them
 * down to the actual storage.
 *
 * The queue keeps track of two pieces of state, a head index, which is the
 * index of the next element to come out of the queue, and a size which is how
 * many valid elements are currently in the queue. Size can increase to, but
 * never exceed, the capacity of the queue.
 *
 * In theory the index may overflow at some point, but since it's a 64 bit
 * value that would take a very long time.
 *
 * @tparam T Type of the elements in the queue
 *
 * @ingroup api_base_utils
 */
template <typename T>
class CircularQueue
{
  protected:
    std::vector<T> data;

    using reference = typename std::vector<T>::reference;
    using const_reference = typename std::vector<T>::const_reference;
    const size_t _capacity;
    size_t _size = 0;
    size_t _head = 1;

    /** Iterator to the circular queue.
     * iterator implementation to provide wrap around indexing which the
     * vector iterator does not.
     */
  public:
    struct iterator
    {
      public:
        CircularQueue* _cq = nullptr;
        size_t _idx = 0;

        /**
         * @ingroup api_base_utils
         */
        iterator(CircularQueue* cq, size_t idx) : _cq(cq), _idx(idx) {}
        iterator() = default;

        /**
         * Iterator Traits
         *
         * @ingroup api_base_utils
         * @{
         */
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using reference = value_type&;
        using const_reference = const value_type&;
        using pointer = value_type*;
        using const_pointer = const value_type*;
        using iterator_category = std::random_access_iterator_tag;
        /** @} */ // end of api_base_utils

        /** Trait reference type
         * iterator satisfies OutputIterator, therefore reference
         * must be T& */
        static_assert(std::is_same_v<reference, T&>,
                "reference type is not assignable as required");

        /**
         * @ingroup api_base_utils
         */
        iterator(const iterator& it) : _cq(it._cq), _idx(it._idx) {}

        /**
         * @ingroup api_base_utils
         */
        iterator&
        operator=(const iterator& it)
        {
            _cq = it._cq;
            _idx = it._idx;
            return *this;
        }

        /**
         * Test dereferenceability.
         * An iterator is dereferenceable if it is pointing to a non-null
         * circular queue, and the index is within the current range of the
         * queue.
         *
         * @ingroup api_base_utils
         */
        bool
        dereferenceable() const
        {
            return _cq != nullptr && _cq->isValidIdx(_idx);
        }

        /** InputIterator. */

        /**
         * Equality operator.
         * Two iterators must point to the same, possibly null, circular
         * queue and the same element on it to be equal.
         *
         * @ingroup api_base_utils
         */
        bool
        operator==(const iterator& that) const
        {
            return _cq == that._cq && _idx == that._idx;
        }

        /**
         * Inequality operator.
         * Conversely, two iterators are different if they both point to
         * different circular queues or they point to different elements.
         *
         * @ingroup api_base_utils
         */
        bool operator!=(const iterator& that) { return !(*this == that); }

        /**
         * Dereference operator.
         *
         * @ingroup api_base_utils
         */
        reference
        operator*()
        {
            // This has to be dereferenceable.
            return (*_cq)[_idx];
        }

        /**
         * @ingroup api_base_utils
         */
        const_reference
        operator*() const
        {
            // This has to be dereferenceable.
            return (*_cq)[_idx];
        }

        /**
         * Dereference operator.
         * Rely on operator* to check for dereferenceability.
         *
         * @ingroup api_base_utils
         */
        pointer operator->() { return &**this; }

        /**
         * @ingroup api_base_utils
         */
        const_pointer operator->() const { return &**this; }

        /**
         * Pre-increment operator.
         *
         * @ingroup api_base_utils
         */
        iterator&
        operator++()
        {
            ++_idx;
            return *this;
        }

        /**
         * Post-increment operator.
         *
         * @ingroup api_base_utils
         */
        iterator
        operator++(int)
        {
            iterator t = *this;
            ++*this;
            return t;
        }

        /** ForwardIterator
         * The multipass guarantee is provided by the reliance on _idx.
         */

        /** BidirectionalIterator requirements. */
      public:
        /**
         * Pre-decrement operator.
         *
         * @ingroup api_base_utils
         */
        iterator&
        operator--()
        {
            /* this has to be decrementable. */
            assert(_cq && _idx > _cq->head());
            --_idx;
            return *this;
        }

        /**
         * Post-decrement operator.
         *
         * @ingroup api_base_utils
         */
        iterator
        operator--(int)
        {
            iterator t = *this;
            --*this;
            return t;
        }

        /**
         * RandomAccessIterator requirements.
         *
         * @ingroup api_base_utils
         */
        iterator&
        operator+=(const difference_type& t)
        {
            _idx += t;
            return *this;
        }

        /**
         * @ingroup api_base_utils
         */
        iterator&
        operator-=(const difference_type& t)
        {
            assert(_cq && _idx >= _cq->head() + t);
            _idx -= t;
            return *this;
        }

        /**
         * Addition operator.
         *
         * @ingroup api_base_utils
         */
        iterator
        operator+(const difference_type& t)
        {
            iterator ret(*this);
            return ret += t;
        }

        /**
         * @ingroup api_base_utils
         */
        friend iterator
        operator+(const difference_type& t, iterator& it)
        {
            iterator ret = it;
            return ret += t;
        }

        /**
         * Substraction operator.
         *
         * @ingroup api_base_utils
         */
        iterator
        operator-(const difference_type& t)
        {
            iterator ret(*this);
            return ret -= t;
        }

        /**
         * @ingroup api_base_utils
         */
        friend iterator
        operator-(const difference_type& t, iterator& it)
        {
            iterator ret = it;
            return ret -= t;
        }

        /**
         * Difference operator.
         * that + ret == this
         *
         * @ingroup api_base_utils
         */
        difference_type
        operator-(const iterator& that)
        {
            return (ssize_t)_idx - (ssize_t)that._idx;
        }

        /**
         * Index operator.
         * The use of * tests for dereferenceability.
         *
         * @ingroup api_base_utils
         */
        template<typename Idx>
        typename std::enable_if_t<std::is_integral_v<Idx>, reference>
        operator[](const Idx& index)
        {
            return *(*this + index);
        }

        /**
         * Comparisons.
         *
         * @ingroup api_base_utils
         */
        bool
        operator<(const iterator& that) const
        {
            return _idx < that._idx;
        }

        /**
         * @ingroup api_base_utils
         */
        bool operator>(const iterator& that) const { return !(*this <= that); }

        /**
         * @ingroup api_base_utils
         */
        bool operator>=(const iterator& that) const { return !(*this < that); }

        /**
         * @ingroup api_base_utils
         */
        bool operator<=(const iterator& that) const { return !(that < *this); }

        /**
         * OutputIterator has no extra requirements.
         */
        size_t idx() const { return _idx; }
    };

  public:
    /**
     * @ingroup api_base_utils
     */
    template <typename Idx>
    typename std::enable_if_t<std::is_integral_v<Idx>, reference>
    operator[](const Idx& index)
    {
        assert(index >= 0);
        return data[index % _capacity];
    }

    template <typename Idx>
    typename std::enable_if_t<std::is_integral_v<Idx>, const_reference>
    operator[](const Idx& index) const
    {
        assert(index >= 0);
        return data[index % _capacity];
    }

    /**
     * @ingroup api_base_utils
     */
    explicit CircularQueue(size_t size=0) : data(size), _capacity(size) {}

    /**
     * Remove all the elements in the queue.
     *
     * Note: This does not actually remove elements from the backing
     * store.
     *
     * @ingroup api_base_utils
     */
    void
    flush()
    {
        _head = 1;
        _size = 0;
    }

    /**
     * Test if the index is in the range of valid elements.
     */
    bool
    isValidIdx(size_t idx) const
    {
        return _head <= idx && idx < (_head + _size);
    }

    /**
     * @ingroup api_base_utils
     */
    reference front() { return (*this)[head()]; }

    /**
     * @ingroup api_base_utils
     */
    reference back() { return (*this)[tail()]; }

    /**
     * @ingroup api_base_utils
     */
    size_t head() const { return _head; }

    /**
     * @ingroup api_base_utils
     */
    size_t tail() const { return _head + _size - 1; }

    /**
     * @ingroup api_base_utils
     */
    size_t capacity() const { return _capacity; }

    /**
     * @ingroup api_base_utils
     */
    size_t size() const { return _size; }

    /** Circularly increase the head pointer.
     * By increasing the head pointer we are removing elements from
     * the begin of the circular queue.
     *
     * @params num_elem number of elements to remove
     *
     * @ingroup api_base_utils
     */
    void
    pop_front(size_t num_elem=1)
    {
        assert(num_elem <= size());
        _head += num_elem;
        _size -= num_elem;
    }

    /**
     * Circularly decrease the tail pointer.
     *
     * @ingroup api_base_utils
     */
    void
    pop_back()
    {
        assert(!empty());
        --_size;
    }

    /**
     * Pushes an element at the end of the queue.
     *
     * @ingroup api_base_utils
     */
    void
    push_back(typename std::vector<T>::value_type val)
    {
        advance_tail();
        back() = val;
    }

    /**
     * Increases the tail by one. This may overwrite the head if the queue is
     * full.
     *
     * @ingroup api_base_utils
     */
    void
    advance_tail()
    {
        if (full())
            ++_head;
        else
            ++_size;
    }

    /**
     * Increases the tail by a specified number of steps. This may overwrite
     * one or more entries starting at the head if the queue is full.
     *
     * @param len Number of steps
     *
     * @ingroup api_base_utils
     */
    void
    advance_tail(size_t len)
    {
        size_t remaining = _capacity - _size;
        if (len > remaining) {
            size_t overflow = len - remaining;
            _head += overflow;
            len -= overflow;
        }
        _size += len;
    }

    /**
     * Is the queue empty?
     *
     * @ingroup api_base_utils
     */
    bool empty() const { return _size == 0; }

    /**
     * Is the queue full?
     * A queue is full if the head is the 0^{th} element and the tail is
     * the (size-1)^{th} element, or if the head is the n^{th} element and
     * the tail the (n-1)^{th} element.
     *
     * @ingroup api_base_utils
     */
    bool full() const { return _size == _capacity; }

    /**
     * Iterators.
     *
     * @ingroup api_base_utils
     */
    iterator begin() { return iterator(this, _head); }

    /* TODO: This should return a const_iterator. */
    /**
     * @ingroup api_base_utils
     */
    iterator
    begin() const
    {
        return iterator(const_cast<CircularQueue*>(this), _head);
    }

    /**
     * @ingroup api_base_utils
     */
    iterator end() { return iterator(this, tail() + 1); }

    /**
     * @ingroup api_base_utils
     */
    iterator
    end() const
    {
        return iterator(const_cast<CircularQueue*>(this), tail() + 1);
    }

    /** Return an iterator to an index in the queue. */
    iterator getIterator(size_t idx) { return iterator(this, idx); }
};

} // namespace gem5

#endif /* __BASE_CIRCULARQUEUE_HH__ */
