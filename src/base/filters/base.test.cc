/*
 * Copyright (c) 2021 Daniel R. Carvalho
 * All rights reserved
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

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <cassert>

#include "base/filters/base.hh"

#define GEM5_DECLARE_FILTER_PARAMS(name) \
    BloomFilterBaseParams name; \
    name.eventq_index = 0; \
    name.size = 3; \
    name.offset_bits = 6; \
    name.num_bits = 1; \
    name.threshold = 1

using namespace gem5;

/** Simulates basic behavior of a bloom filter. */
class TestFilter : public bloom_filter::Base
{
  public:
    using bloom_filter::Base::Base;

    void
    set(Addr addr) override
    {
        assert(addr < filter.size());
        filter[addr]++;
    }

    int
    getCount(Addr addr) const override
    {
        assert(addr < filter.size());
        return filter[addr];
    }
};

/** Test that a filter is initialized in a cleared state. */
TEST(BloomFilterBaseTest, Construct)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);
    ASSERT_EQ(filter.getTotalCount(), 0);
    for (int i = 0; i < params.size; i++) {
        ASSERT_FALSE(filter.isSet(i));
    }
}

/**
 * Test that setting a single entry when the threshold is 1 will only set
 * that entry, and no other entry.
 */
TEST(BloomFilterBaseTest, SingleIsSet)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);
    ASSERT_EQ(filter.getTotalCount(), 0);

    filter.set(0);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_TRUE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));

    filter.clear();
    ASSERT_EQ(filter.getTotalCount(), 0);
    filter.set(1);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));

    filter.clear();
    ASSERT_EQ(filter.getTotalCount(), 0);
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));
}

/**
 * Test that isSet works for multiple simultaneously set entries by
 * simultaneously saturating different entries at the same time.
 */
TEST(BloomFilterBaseTest, MultipleIsSet)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);
    ASSERT_EQ(filter.getTotalCount(), 0);

    filter.set(0);
    ASSERT_EQ(filter.getTotalCount(), 1);
    filter.set(1);
    ASSERT_EQ(filter.getTotalCount(), 2);
    ASSERT_TRUE(filter.isSet(0));
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));

    filter.clear();
    ASSERT_EQ(filter.getTotalCount(), 0);
    filter.set(1);
    ASSERT_EQ(filter.getTotalCount(), 1);
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 2);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));

    filter.clear();
    ASSERT_EQ(filter.getTotalCount(), 0);
    filter.set(0);
    ASSERT_EQ(filter.getTotalCount(), 1);
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 2);
    ASSERT_TRUE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));

    filter.clear();
    ASSERT_EQ(filter.getTotalCount(), 0);
    filter.set(0);
    ASSERT_EQ(filter.getTotalCount(), 1);
    filter.set(1);
    ASSERT_EQ(filter.getTotalCount(), 2);
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 3);
    ASSERT_TRUE(filter.isSet(0));
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));
}

/**
 * Test that isSet takes the threshold into consideration. This test
 * increases the number of bits in the filter's entries to be able to
 * raise the threshold at which an entry is considered as set.
 */
TEST(BloomFilterBaseTest, SingleIsSetThreshold)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    params.num_bits = 2;
    params.threshold = 2;
    TestFilter filter(params);
    ASSERT_EQ(filter.getTotalCount(), 0);

    filter.set(0);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));
    filter.set(0);
    ASSERT_EQ(filter.getTotalCount(), 2);
    ASSERT_TRUE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));

    filter.clear();
    filter.set(1);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));
    filter.set(1);
    ASSERT_EQ(filter.getTotalCount(), 2);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));

    filter.clear();
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 2);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));

    // Setting different entries once should not make any of them
    // reach the threshold
    filter.clear();
    filter.set(0);
    filter.set(1);
    filter.set(2);
    ASSERT_EQ(filter.getTotalCount(), 3);
    ASSERT_FALSE(filter.isSet(0));
    ASSERT_FALSE(filter.isSet(1));
    ASSERT_FALSE(filter.isSet(2));
}

/** Test that merging two empty bloom filters results in an empty filter. */
TEST(BloomFilterBaseTest, MergeBothEmpty)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);
    TestFilter filter2(params);

    filter.merge(&filter2);
    ASSERT_EQ(filter.getTotalCount(), 0);
    ASSERT_EQ(filter2.getTotalCount(), 0);
}

/**
 * Test that merging a populated filter with an empty filter does not modify
 * any of the filters.
 */
TEST(BloomFilterBaseTest, MergeWithEmpty)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);
    filter.set(1);

    TestFilter filter2(params);

    filter.merge(&filter2);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_EQ(filter2.getTotalCount(), 0);
}

/**
 * Test that merging an empty filter with a populated filter results in
 * two equal filters.
 */
TEST(BloomFilterBaseTest, MergeWithEmpty2)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);

    TestFilter filter2(params);
    filter2.set(1);

    filter.merge(&filter2);
    ASSERT_EQ(filter.getTotalCount(), 1);
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_EQ(filter2.getTotalCount(), 1);
    ASSERT_TRUE(filter.isSet(1));
}

/**
 * Test merging two filters with intersecting entries. The caller is modified,
 * but the other filter is not.
 */
TEST(BloomFilterBaseTest, MergeNoIntersection)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    params.size = 10;

    TestFilter filter(params);
    filter.set(1);
    filter.set(2);
    filter.set(5);
    filter.set(8);

    TestFilter filter2(params);
    filter2.set(3);
    filter2.set(4);
    filter2.set(9);

    filter.merge(&filter2);
    ASSERT_EQ(filter.getTotalCount(), 7);
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));
    ASSERT_TRUE(filter.isSet(3));
    ASSERT_TRUE(filter.isSet(4));
    ASSERT_TRUE(filter.isSet(5));
    ASSERT_TRUE(filter.isSet(8));
    ASSERT_TRUE(filter.isSet(9));
    ASSERT_EQ(filter2.getTotalCount(), 3);
    ASSERT_TRUE(filter2.isSet(3));
    ASSERT_TRUE(filter2.isSet(4));
    ASSERT_TRUE(filter2.isSet(9));
}

/** Test merging two filters with insersecting entries and threshold at 1. */
TEST(BloomFilterBaseTest, MergeIntersectionThreshold1)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    params.size = 10;

    TestFilter filter(params);
    filter.set(1);
    filter.set(2);
    filter.set(5);
    filter.set(8);

    TestFilter filter2(params);
    filter2.set(3);
    filter2.set(5);
    filter2.set(9);

    filter.merge(&filter2);
    ASSERT_EQ(filter.getTotalCount(), 6);
    ASSERT_TRUE(filter.isSet(1));
    ASSERT_TRUE(filter.isSet(2));
    ASSERT_TRUE(filter.isSet(3));
    ASSERT_TRUE(filter.isSet(5));
    ASSERT_TRUE(filter.isSet(8));
    ASSERT_TRUE(filter.isSet(9));
    ASSERT_EQ(filter2.getTotalCount(), 3);
    ASSERT_TRUE(filter2.isSet(3));
    ASSERT_TRUE(filter2.isSet(5));
    ASSERT_TRUE(filter2.isSet(9));
}

/**
 * Test merging two filters with insersecting entries and threshold at 2.
 * One entry is populated so that it only reaches the threshold after merging.
 * One entry is populated so that when merged it will become saturated.
 */
TEST(BloomFilterBaseTest, MergeIntersectionThreshold2)
{
    GEM5_DECLARE_FILTER_PARAMS(params);
    params.size = 10;
    params.num_bits = 2;
    params.threshold = 2;

    TestFilter filter(params);
    filter.set(1);
    filter.set(2);
    filter.set(5);
    filter.set(5);
    filter.set(8);

    TestFilter filter2(params);
    filter2.set(2);
    filter2.set(5);
    filter2.set(5);
    filter2.set(5);
    filter2.set(9);

    filter.merge(&filter2);
    // 1 one, 2 twos, 3 fives (saturated), 1 eight, 1 nine
    ASSERT_EQ(filter.getTotalCount(), 8);
    ASSERT_TRUE(filter.isSet(2));
    ASSERT_TRUE(filter.isSet(5));
    ASSERT_EQ(filter2.getTotalCount(), 5);
    ASSERT_FALSE(filter2.isSet(2));
    ASSERT_TRUE(filter2.isSet(5));
}

/** Test that trying to merge filters of different sizes fails. */
TEST(BloomFilterBaseDeathTest, MergeDifferent)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    GEM5_DECLARE_FILTER_PARAMS(params);
    TestFilter filter(params);

    BloomFilterBaseParams params2;
    params2.eventq_index = params.eventq_index;
    params2.size = params.size + 1;
    params2.offset_bits = params.offset_bits;
    params2.num_bits = params.num_bits;
    params2.threshold = params.threshold;
    TestFilter filter2(params2);

    ASSERT_DEATH(filter.merge(&filter2), "");
}

#undef GEM5_DECLARE_FILTER_PARAMS
