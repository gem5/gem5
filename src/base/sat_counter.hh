/*
 * Copyright (c) 2019 Inria
 * All rights reserved.
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
 * Copyright (c) 2005-2006 The Regents of The University of Michigan
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

#ifndef __BASE_SAT_COUNTER_HH__
#define __BASE_SAT_COUNTER_HH__

#include <cassert>
#include <cstdint>

#include "base/logging.hh"
#include "base/types.hh"

/**
 * Implements an n bit saturating counter and provides methods to
 * increment, decrement, and read it.
 */
class SatCounter
{
  public:
    /** The default constructor should never be used. */
    SatCounter() = delete;

    /**
     * Constructor for the counter. The explicit keyword is used to make
     * sure the user does not assign a number to the counter thinking it
     * will be used as a counter value when it is in fact used as the number
     * of bits.
     *
     * @param bits How many bits the counter will have.
     * @param initial_val Starting value for the counter.
     */
    explicit SatCounter(unsigned bits, uint8_t initial_val = 0)
        : initialVal(initial_val), maxVal((1 << bits) - 1),
          counter(initial_val)
    {
        fatal_if(bits > 8*sizeof(uint8_t),
                 "Number of bits exceeds counter size");
        fatal_if(initial_val > maxVal,
                 "Saturating counter's Initial value exceeds max value.");
    }

    /** Copy constructor. */
    SatCounter(const SatCounter& other)
        : initialVal(other.initialVal), maxVal(other.maxVal),
          counter(other.counter)
    {
    }

    /** Copy assignment. */
    SatCounter& operator=(const SatCounter& other) {
        if (this != &other) {
            SatCounter temp(other);
            this->swap(temp);
        }
        return *this;
    }

    /** Move constructor. */
    SatCounter(SatCounter&& other)
    {
        initialVal = other.initialVal;
        maxVal = other.maxVal;
        counter = other.counter;
        SatCounter temp(0);
        other.swap(temp);
    }

    /** Move assignment. */
    SatCounter& operator=(SatCounter&& other) {
        if (this != &other) {
            initialVal = other.initialVal;
            maxVal = other.maxVal;
            counter = other.counter;
            SatCounter temp(0);
            other.swap(temp);
        }
        return *this;
    }

    /**
     * Swap the contents of every member of the class. Used for the default
     * copy-assignment created by the compiler.
     *
     * @param other The other object to swap contents with.
     */
    void
    swap(SatCounter& other)
    {
        std::swap(initialVal, other.initialVal);
        std::swap(maxVal, other.maxVal);
        std::swap(counter, other.counter);
    }

    /** Pre-increment operator. */
    SatCounter&
    operator++()
    {
        if (counter < maxVal) {
            ++counter;
        }
        return *this;
    }

    /** Post-increment operator. */
    SatCounter
    operator++(int)
    {
        SatCounter old_counter = *this;
        ++*this;
        return old_counter;
    }

    /** Pre-decrement operator. */
    SatCounter&
    operator--()
    {
        if (counter > 0) {
            --counter;
        }
        return *this;
    }

    /** Post-decrement operator. */
    SatCounter
    operator--(int)
    {
        SatCounter old_counter = *this;
        --*this;
        return old_counter;
    }

    /** Shift-right-assignment. */
    SatCounter&
    operator>>=(const int& shift)
    {
        assert(shift >= 0);
        this->counter >>= shift;
        return *this;
    }

    /** Shift-left-assignment. */
    SatCounter&
    operator<<=(const int& shift)
    {
        assert(shift >= 0);
        this->counter <<= shift;
        if (this->counter > maxVal) {
            this->counter = maxVal;
        }
        return *this;
    }

    /** Add-assignment. */
    SatCounter&
    operator+=(const int& value)
    {
        if (value >= 0) {
            if (maxVal - this->counter >= value) {
                this->counter += value;
            } else {
                this->counter = maxVal;
            }
        } else {
            *this -= -value;
        }
        return *this;
    }

    /** Subtract-assignment. */
    SatCounter&
    operator-=(const int& value)
    {
        if (value >= 0) {
            if (this->counter > value) {
                this->counter -= value;
            } else {
                this->counter = 0;
            }
        } else {
            *this += -value;
        }
        return *this;
    }

    /**
     * Read the counter's value.
     */
    operator uint8_t() const { return counter; }

    /** Reset the counter to its initial value. */
    void reset() { counter = initialVal; }

    /**
     * Calculate saturation percentile of the current counter's value
     * with regard to its maximum possible value.
     *
     * @return A value between 0.0 and 1.0 to indicate which percentile of
     *         the maximum value the current value is.
     */
    double calcSaturation() const { return (double) counter / maxVal; }

    /**
     * Whether the counter has achieved its maximum value or not.
     *
     * @return True if the counter saturated.
     */
    bool isSaturated() const { return counter == maxVal; }

    /**
     * Saturate the counter.
     *
     * @return The value added to the counter to reach saturation.
     */
    uint8_t saturate()
    {
        const uint8_t diff = maxVal - counter;
        counter = maxVal;
        return diff;
    }

  private:
    uint8_t initialVal;
    uint8_t maxVal;
    uint8_t counter;
};

#endif // __BASE_SAT_COUNTER_HH__
