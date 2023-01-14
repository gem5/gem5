/*
 * Copyright (c) 2021 Daniel R. Carvalho
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __BASE_STATS_STORAGE_HH__
#define __BASE_STATS_STORAGE_HH__

#include <cassert>
#include <cmath>

#include "base/cast.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/stats/types.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace statistics
{

struct StorageParams
{
    virtual ~StorageParams() = default;
};

/**
 * Templatized storage and interface for a simple scalar stat.
 */
class StatStor
{
  private:
    /** The statistic value. */
    Counter data;

  public:
    struct Params : public StorageParams {};

    /**
     * Builds this storage element and calls the base constructor of the
     * datatype.
     */
    StatStor(const StorageParams* const storage_params)
        : data(Counter())
    { }

    /**
     * The the stat to the given value.
     * @param val The new value.
     */
    void set(Counter val) { data = val; }

    /**
     * Increment the stat by the given value.
     * @param val The new value.
     */
    void inc(Counter val) { data += val; }

    /**
     * Decrement the stat by the given value.
     * @param val The new value.
     */
    void dec(Counter val) { data -= val; }

    /**
     * Return the value of this stat as its base type.
     * @return The value of this stat.
     */
    Counter value() const { return data; }

    /**
     * Return the value of this stat as a result type.
     * @return The value of this stat.
     */
    Result result() const { return (Result)data; }

    /**
     * Prepare stat data for dumping or serialization
     */
    void prepare(const StorageParams* const storage_params) { }

    /**
     * Reset stat value to default
     */
    void reset(const StorageParams* const storage_params) { data = Counter(); }

    /**
     * @return true if zero value
     */
    bool zero() const { return data == Counter(); }
};

/**
 * Templatized storage and interface to a per-tick average stat. This keeps
 * a current count and updates a total (count * ticks) when this count
 * changes. This allows the quick calculation of a per tick count of the item
 * being watched. This is good for keeping track of residencies in structures
 * among other things.
 */
class AvgStor
{
  private:
    /** The current count. */
    Counter current;
    /** The tick of the last reset */
    Tick lastReset;
    /** The total count for all tick. */
    mutable Result total;
    /** The tick that current last changed. */
    mutable Tick last;

  public:
    struct Params : public StorageParams {};

    /**
     * Build and initializes this stat storage.
     */
    AvgStor(const StorageParams* const storage_params)
        : current(0), lastReset(0), total(0), last(0)
    { }

    /**
     * Set the current count to the one provided, update the total and last
     * set values.
     * @param val The new count.
     */
    void
    set(Counter val)
    {
        total += current * (curTick() - last);
        last = curTick();
        current = val;
    }

    /**
     * Increment the current count by the provided value, calls set.
     * @param val The amount to increment.
     */
    void inc(Counter val) { set(current + val); }

    /**
     * Deccrement the current count by the provided value, calls set.
     * @param val The amount to decrement.
     */
    void dec(Counter val) { set(current - val); }

    /**
     * Return the current count.
     * @return The current count.
     */
    Counter value() const { return current; }

    /**
     * Return the current average.
     * @return The current average.
     */
    Result
    result() const
    {
        assert(last == curTick());
        return (Result)(total + current) / (Result)(curTick() - lastReset + 1);
    }

    /**
     * @return true if zero value
     */
    bool zero() const { return total == 0.0; }

    /**
     * Prepare stat data for dumping or serialization
     */
    void
    prepare(const StorageParams* const storage_params)
    {
        total += current * (curTick() - last);
        last = curTick();
    }

    /**
     * Reset stat value to default
     */
    void
    reset(const StorageParams* const storage_params)
    {
        total = 0.0;
        last = curTick();
        lastReset = curTick();
    }

};

/** The parameters for a distribution stat. */
struct DistParams : public StorageParams
{
    const DistType type;
    DistParams(DistType t) : type(t) {}
};

/**
 * Templatized storage and interface for a distribution stat. A distribution
 * uses buckets to keep track of values within a given range. All other
 * values, although accounted for on the overall calculations, are not tracked
 * in buckets themselves; two special counters, underflow and overflow store
 * the number of occurrences of such values.
 */
class DistStor
{
  private:
    /** The minimum value to track. */
    Counter min_track;
    /** The maximum value to track. */
    Counter max_track;
    /** The number of entries in each bucket. */
    Counter bucket_size;

    /** The smallest value sampled. */
    Counter min_val;
    /** The largest value sampled. */
    Counter max_val;
    /** The number of values sampled less than min. */
    Counter underflow;
    /** The number of values sampled more than max. */
    Counter overflow;
    /** The current sum. */
    Counter sum;
    /** The sum of squares. */
    Counter squares;
    /** The number of samples. */
    Counter samples;
    /** Counter for each bucket. */
    VCounter cvec;

  public:
    /** The parameters for a distribution stat. */
    struct Params : public DistParams
    {
        /** The minimum value to track. */
        Counter min;
        /** The maximum value to track. */
        Counter max;
        /** The number of entries in each bucket. */
        Counter bucket_size;
        /** The number of buckets. Equal to (max-min)/bucket_size. */
        size_type buckets;

        Params(Counter _min, Counter _max, Counter _bucket_size)
          : DistParams(Dist), min(_min), max(_max), bucket_size(_bucket_size),
            buckets(0)
        {
            fatal_if(bucket_size <= 0,
                "Bucket size (%f) must be greater than zero", bucket_size);
            warn_if(std::floor((max - min + 1.0) / bucket_size) !=
                std::ceil((max - min + 1.0) / bucket_size),
                "Bucket size (%f) does not divide range [%f:%f] into equal-" \
                "sized buckets. Rounding up.", bucket_size, min + 1.0, max);

            buckets = std::ceil((max - min + 1.0) / bucket_size);
        }
    };

    DistStor(const StorageParams* const storage_params)
        : cvec(safe_cast<const Params *>(storage_params)->buckets)
    {
        reset(storage_params);
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void sample(Counter val, int number);

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     */
    size_type size() const { return cvec.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @return True if any values have been sampled.
     */
    bool
    zero() const
    {
        return samples == Counter();
    }

    void
    prepare(const StorageParams* const storage_params, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(storage_params);

        assert(params->type == Dist);
        data.type = params->type;
        data.min = params->min;
        data.max = params->max;
        data.bucket_size = params->bucket_size;

        data.min_val = (min_val == CounterLimits::max()) ? 0 : min_val;
        data.max_val = (max_val == CounterLimits::min()) ? 0 : max_val;
        data.underflow = underflow;
        data.overflow = overflow;

        data.cvec.resize(params->buckets);
        for (off_type i = 0; i < params->buckets; ++i)
            data.cvec[i] = cvec[i];

        data.sum = sum;
        data.squares = squares;
        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(const StorageParams* const storage_params)
    {
        const Params *params = safe_cast<const Params *>(storage_params);
        min_track = params->min;
        max_track = params->max;
        bucket_size = params->bucket_size;

        min_val = CounterLimits::max();
        max_val = CounterLimits::min();
        underflow = Counter();
        overflow = Counter();

        size_type size = cvec.size();
        for (off_type i = 0; i < size; ++i)
            cvec[i] = Counter();

        sum = Counter();
        squares = Counter();
        samples = Counter();
    }
};

/**
 * Templatized storage and interface for a histogram stat.
 *
 * The number of buckets is fixed on initialization; however, the bucket size
 * isn't. That means that when samples that are outside the current range are
 * seen, the bucket size will be increased so that each bucket can hold a
 * bigger range of values. When that happens, the bucket's contents are re-
 * located.
 *
 * The min and max bucket values can only be, respectively, decreased and
 * increased when sampling. If this wasn't true, samples that were previously
 * within the buclet range could not be anymore within the valid range, making
 * the storage's state incoherent. These values are set back to their initial
 * states on reset().
 *
 * The bucket range always is zero-centric. While the storage does not
 * contain negative values the bucket range will keep its lower bound at
 * zero, doubling the upper bound when needed; However, as soon a negative
 * value is sampled, zero becomes the lower bound of the middle (rounded up)
 * bucket. Although this means that the histogram will not be symmetric if
 * negative values are sampled, it makes it possible to grow the buckets
 * without keeping track of the individual values.
 *
 * This happens because if zero was not a lower or upper bound, when its
 * value was doubled, the lower and upper bound of the bucket containing
 * zero would intersect with middle values of the previous and next buckets.
 * For example, if the bucket containing zero has range [-2,2[, therefore
 * its neighbor buckets would have ranges at [-6,-2[ and [2,6[. When the
 * buckets are grown, the zero bucket would grow its range to [-4,4[, which
 * cannot be easily extracted from the neighor buckets.
 */
class HistStor
{
  private:
    /** Lower bound of the first bucket's range. */
    Counter min_bucket;
    /** Lower bound of the last bucket's range. */
    Counter max_bucket;
    /** The number of entries in each bucket. */
    Counter bucket_size;

    /** The current sum. */
    Counter sum;
    /** The sum of logarithm of each sample, used to compute geometric mean. */
    Counter logs;
    /** The sum of squares. */
    Counter squares;
    /** The number of samples. */
    Counter samples;
    /** Counter for each bucket. */
    VCounter cvec;

    /**
     * Given a bucket size B, and a range of values [0, N], this function
     * doubles the bucket size to double the range of values towards the
     * positive infinite; that is, double the upper range of this storage
     * so that the range becomes [0, 2*N].
     *
     * Because the bucket size is doubled, the buckets contents are rearranged,
     * since the original range of values is mapped to the lower half buckets.
     */
    void growUp();

    /**
     * Given a bucket size B, and a range of values [M, N], where M < 0, this
     * function doubles the bucket size to double the range of values towards
     * both positive and negative infinites; that is, it doubles both the lower
     * and the upper range of this storage so that the range becomes
     * [2*M, 2*N].
     *
     * Because the bucket size is doubled, the buckets contents are
     * rearranged, and the original range of values are redistributed to free
     * buckets for the newly appended ranges.
     */
    void growOut();

    /**
     * Given a bucket size B, and a range of values [0, N], this function
     * doubles the bucket size to double the range of values towards the
     * negative infinite; that is, it doubles the lower range of this
     * storage so that the middle buckes contaihs zero as a lower bound. As
     * such, the storage range becomes [-N, N+B] if there is an odd number
     * of buckets, and [-N-B, N+B] if there is an even number of buckets.
     *
     * Because the bucket size is doubled, the buckets contents are
     * rearranged, and the original range of values are redistributed to free
     * buckets for the newly appended ranges.
     */
    void growDown();

  public:
    /** The parameters for a distribution stat. */
    struct Params : public DistParams
    {
        /** The number of buckets. */
        size_type buckets;

        Params(size_type _buckets)
          : DistParams(Hist)
        {
            fatal_if(_buckets < 2,
                "There must be at least two buckets in a histogram");
            buckets = _buckets;
        }
    };

    HistStor(const StorageParams* const storage_params)
        : cvec(safe_cast<const Params *>(storage_params)->buckets)
    {
        reset(storage_params);
    }

    /**
     * Adds the contents of the given storage to this storage.
     * @param other The other storage to be added.
     */
    void add(HistStor *other);

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void sample(Counter val, int number);

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     */
    size_type size() const { return cvec.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @return True if any values have been sampled.
     */
    bool
    zero() const
    {
        return samples == Counter();
    }

    void
    prepare(const StorageParams* const storage_params, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(storage_params);

        assert(params->type == Hist);
        data.type = params->type;
        data.min = min_bucket;
        data.max = max_bucket + bucket_size - 1;
        data.bucket_size = bucket_size;

        data.min_val = min_bucket;
        data.max_val = max_bucket;

        int buckets = params->buckets;
        data.cvec.resize(buckets);
        for (off_type i = 0; i < buckets; ++i)
            data.cvec[i] = cvec[i];

        data.sum = sum;
        data.logs = logs;
        data.squares = squares;
        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(const StorageParams* const storage_params)
    {
        const Params *params = safe_cast<const Params *>(storage_params);
        min_bucket = 0;
        max_bucket = params->buckets - 1;
        bucket_size = 1;

        size_type size = cvec.size();
        for (off_type i = 0; i < size; ++i)
            cvec[i] = Counter();

        sum = Counter();
        squares = Counter();
        samples = Counter();
        logs = Counter();
    }
};

/**
 * Templatized storage and interface for a distribution that calculates mean
 * and variance.
 */
class SampleStor
{
  private:
    /** The current sum. */
    Counter sum;
    /** The sum of squares. */
    Counter squares;
    /** The number of samples. */
    Counter samples;

  public:
    struct Params : public DistParams
    {
        Params() : DistParams(Deviation) {}
    };

    /**
     * Create and initialize this storage.
     */
    SampleStor(const StorageParams* const storage_params)
        : sum(Counter()), squares(Counter()), samples(Counter())
    { }

    /**
     * Add a value the given number of times to this running average.
     * Update the running sum and sum of squares, increment the number of
     * values seen by the given number.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        sum += val * number;
        squares += val * val * number;
        samples += number;
    }

    /**
     * Return the number of entries in this stat, 1
     * @return 1.
     */
    size_type size() const { return 1; }

    /**
     * Return true if no samples have been added.
     * @return True if no samples have been added.
     */
    bool zero() const { return samples == Counter(); }

    void
    prepare(const StorageParams* const storage_params, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(storage_params);

        assert(params->type == Deviation);
        data.type = params->type;
        data.sum = sum;
        data.squares = squares;
        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(const StorageParams* const storage_params)
    {
        sum = Counter();
        squares = Counter();
        samples = Counter();
    }
};

/**
 * Templatized storage for distribution that calculates per tick mean and
 * variance.
 */
class AvgSampleStor
{
  private:
    /** Current total. */
    Counter sum;
    /** Current sum of squares. */
    Counter squares;

  public:
    struct Params : public DistParams
    {
        Params() : DistParams(Deviation) {}
    };

    /**
     * Create and initialize this storage.
     */
    AvgSampleStor(const StorageParams* const storage_params)
        : sum(Counter()), squares(Counter())
    {}

    /**
     * Add a value to the distribution for the given number of times.
     * Update the running sum and sum of squares.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        sum += val * number;
        squares += val * val * number;
    }

    /**
     * Return the number of entries, in this case 1.
     * @return 1.
     */
    size_type size() const { return 1; }

    /**
     * Return true if no samples have been added.
     * @return True if the sum is zero.
     */
    bool zero() const { return sum == Counter(); }

    void
    prepare(const StorageParams* const storage_params, DistData &data)
    {
        const Params *params = safe_cast<const Params *>(storage_params);

        assert(params->type == Deviation);
        data.type = params->type;
        data.sum = sum;
        data.squares = squares;
        data.samples = curTick();
    }

    /**
     * Reset stat value to default
     */
    void
    reset(const StorageParams* const storage_params)
    {
        sum = Counter();
        squares = Counter();
    }
};

/**
 * Templatized storage and interface for a sparse histogram stat. There
 * is no actual limit on the number of buckets, and each of them has a size
 * of 1, meaning that samples are individually recorded, and there is no
 * need to keep track of the samples that occur in between two distant
 * sampled values.
 */
class SparseHistStor
{
  private:
    /** Counter for number of samples */
    Counter samples;
    /** Counter for each bucket. */
    MCounter cmap;

  public:
    /** The parameters for a sparse histogram stat. */
    struct Params : public DistParams
    {
        Params() : DistParams(Hist) {}
    };

    SparseHistStor(const StorageParams* const storage_params)
    {
        reset(storage_params);
    }

    /**
     * Add a value to the distribution for the given number of times.
     * @param val The value to add.
     * @param number The number of times to add the value.
     */
    void
    sample(Counter val, int number)
    {
        cmap[val] += number;
        samples += number;
    }

    /**
     * Return the number of buckets in this distribution.
     * @return the number of buckets.
     */
    size_type size() const { return cmap.size(); }

    /**
     * Returns true if any calls to sample have been made.
     * @return True if any values have been sampled.
     */
    bool
    zero() const
    {
        return samples == Counter();
    }

    void
    prepare(const StorageParams* const storage_params, SparseHistData &data)
    {
        MCounter::iterator it;
        data.cmap.clear();
        for (it = cmap.begin(); it != cmap.end(); it++) {
            data.cmap[(*it).first] = (*it).second;
        }

        data.samples = samples;
    }

    /**
     * Reset stat value to default
     */
    void
    reset(const StorageParams* const storage_params)
    {
        cmap.clear();
        samples = 0;
    }
};

} // namespace statistics
} // namespace gem5

#endif // __BASE_STATS_STORAGE_HH__
