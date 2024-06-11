/*
 * Copyright 2021 Google, Inc.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <initializer_list>
#include <list>
#include <memory>
#include <ostream>
#include <vector>

#include "base/cprintf.hh"
#include "base/gtest/logging.hh"
#include "mem/translation_gen.hh"

using testing::HasSubstr;
using testing::Pointwise;
using namespace gem5;

namespace gem5
{

// A dummy fault class so we have something to return from failed translations.
class FaultBase {};

Fault dummyFault1 = std::make_shared<gem5::FaultBase>();
Fault dummyFault2 = std::make_shared<gem5::FaultBase>();

std::ostream &
operator<<(std::ostream &os, const TranslationGen::Range &range)
{
    if (range.fault == dummyFault1)
        ccprintf(os, "%#x=>fault1", range.vaddr);
    else if (range.fault == dummyFault2)
        ccprintf(os, "%#x=>fault2", range.vaddr);
    else
        ccprintf(os, "%#x=>%#x [%#x]", range.vaddr, range.paddr, range.size);
    return os;
}

} // namespace gem5

using RangeList = std::list<TranslationGen::Range>;

// Compare ranges which are returned by the generator.
MATCHER(GenRangeEq, "")
{
    const auto &[actual, expected] = arg;
    // vaddr and fault should match no matter what.
    if (actual.vaddr != expected.vaddr || actual.fault != expected.fault)
        return false;
    if (expected.fault) {
        // If there was a fault, paddr and size don't matter.
        return true;
    } else {
        // If there was no fault, check paddr and size.
        return actual.paddr == expected.paddr && actual.size == expected.size;
    }
}

// Compare ranges which are sent to the translate function.
MATCHER(TransRangeEq, "")
{
    const auto &[actual, expected] = arg;
    // Only the vaddr and size fields are inputs to the translate function.
    return actual.vaddr == expected.vaddr && actual.size == expected.size;
}

class TestTranslationGen : public TranslationGen
{
  public:
    // Results to return from the translate function, one by one.
    const std::vector<Range> results;
    // All the Ranges which were passed into translate from the generator.
    mutable RangeList args;

  private:
    // Where we are in the results vector.
    mutable std::vector<Range>::const_iterator resultPos;

  public:
    TestTranslationGen(Addr new_start, Addr new_size,
            std::initializer_list<Range> ranges={}) :
        TranslationGen(new_start, new_size), results(ranges),
        resultPos(results.begin())
    {}

    void
    translate(Range &range) const override
    {
        // First, record what range we were asked to translate.
        args.emplace_back(range);
        // Then, if we're not out of results to return...
        if (resultPos != results.end()) {
            // Record the fault we're supposed to return, if any.
            range.fault = resultPos->fault;
            if (!range.fault) {
                // If there wasn't a fault, size and paddr are meaningful.
                range.size = resultPos->size;
                range.paddr = resultPos->paddr;
                range.flags = resultPos->flags;
            }
            // Advance to the next result.
            resultPos++;
        }
    }
};

TEST(TranslationGen, Accessors)
{
    TestTranslationGen gen1(0x10000, 0x20000);

    EXPECT_EQ(gen1.start(), 0x10000);
    EXPECT_EQ(gen1.size(), 0x20000);

    TestTranslationGen gen2(0x3456, 0x6543);

    EXPECT_EQ(gen2.start(), 0x3456);
    EXPECT_EQ(gen2.size(), 0x6543);
}

TEST(TranslationGen, BeginAndEnd)
{
    TestTranslationGen gen1(0x10000, 0x20000);

    EXPECT_NE(gen1.begin(), gen1.end());
    EXPECT_EQ(gen1.begin(), gen1.begin());
    EXPECT_EQ(gen1.end(), gen1.end());

    TestTranslationGen gen2(0x30000, 0x40000);

    EXPECT_NE(gen1.begin(), gen2.begin());
    EXPECT_EQ(gen1.end(), gen2.end());
}

TEST(TranslationGen, SuccessfulTwoStep)
{
    TestTranslationGen gen(0x10000, 0x10000, {
            // Results for translate.
            {0x0, 0x8000, 0x30000, {}, NoFault},
            {0x0, 0x8000, 0x40000, {}, NoFault}
    });

    RangeList range_list;
    for (const auto &range: gen)
        range_list.emplace_back(range);

    // What the generator should return.
    const RangeList expected_gen{
        {0x10000, 0x8000, 0x30000, {}, NoFault},
        {0x18000, 0x8000, 0x40000, {}, NoFault}
    };
    EXPECT_THAT(range_list, Pointwise(GenRangeEq(), expected_gen));

    // What the generator should have been asked to translate.
    const RangeList expected_trans{
        {0x10000, 0x10000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault}
    };
    EXPECT_THAT(gen.args, Pointwise(TransRangeEq(), expected_trans));
}

TEST(TranslationGen, RetryOnFault)
{
    TestTranslationGen gen(0x10000, 0x10000, {
            // Results for translate.
            {0x0, 0x8000, 0x30000, {}, NoFault},
            {0x0, 0x0, 0x0, {}, dummyFault1},
            {0x0, 0x8000, 0x40000, {}, NoFault}
    });

    RangeList range_list;
    for (const auto &range: gen)
        range_list.emplace_back(range);

    // What the generator should return.
    const RangeList expected_gen{
        {0x10000, 0x8000, 0x30000, {}, NoFault},
        {0x18000, 0x0, 0x0, {}, dummyFault1},
        {0x18000, 0x8000, 0x40000, {}, NoFault}
    };
    EXPECT_THAT(range_list, Pointwise(GenRangeEq(), expected_gen));

    // What the generator should have been asked to translate.
    const RangeList expected_trans{
        {0x10000, 0x10000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault}
    };
    EXPECT_THAT(gen.args, Pointwise(TransRangeEq(), expected_trans));
}

TEST(TranslationGen, RetryTwiceOnFault)
{
    TestTranslationGen gen(0x10000, 0x10000, {
            // Results for translate.
            {0x0, 0x8000, 0x30000, {}, NoFault},
            {0x0, 0x0, 0x0, {}, dummyFault1},
            {0x0, 0x0, 0x0, {}, dummyFault2},
            {0x0, 0x8000, 0x40000, {}, NoFault}
    });

    RangeList range_list;
    for (const auto &range: gen)
        range_list.emplace_back(range);

    // What the generator should return.
    const RangeList expected_gen{
        {0x10000, 0x8000, 0x30000, {}, NoFault},
        {0x18000, 0x0, 0x0, {}, dummyFault1},
        {0x18000, 0x0, 0x0, {}, dummyFault2},
        {0x18000, 0x8000, 0x40000, {}, NoFault}
    };
    EXPECT_THAT(range_list, Pointwise(GenRangeEq(), expected_gen));

    // What the generator should have been asked to translate.
    const RangeList expected_trans{
        {0x10000, 0x10000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault}
    };
    EXPECT_THAT(gen.args, Pointwise(TransRangeEq(), expected_trans));
}

TEST(TranslationGen, FaultAtStart)
{
    TestTranslationGen gen(0x10000, 0x10000, {
            // Results for translate.
            {0x0, 0x0, 0x0, {}, dummyFault1},
            {0x0, 0x8000, 0x30000, {}, NoFault},
            {0x0, 0x8000, 0x40000, {}, NoFault}
    });

    RangeList range_list;
    for (const auto &range: gen)
        range_list.emplace_back(range);

    // What the generator should return.
    const RangeList expected_gen{
        {0x10000, 0x0, 0x0, {}, dummyFault1},
        {0x10000, 0x8000, 0x30000, {}, NoFault},
        {0x18000, 0x8000, 0x40000, {}, NoFault}
    };
    EXPECT_THAT(range_list, Pointwise(GenRangeEq(), expected_gen));

    // What the generator should have been asked to translate.
    const RangeList expected_trans{
        {0x10000, 0x10000, 0x0, {}, NoFault},
        {0x10000, 0x10000, 0x0, {}, NoFault},
        {0x18000, 0x8000, 0x0, {}, NoFault}
    };
    EXPECT_THAT(gen.args, Pointwise(TransRangeEq(), expected_trans));
}

TEST(TranslationGen, FaultInMiddle)
{
    TestTranslationGen gen(0x10000, 0x18000, {
            // Results for translate.
            {0x0, 0x8000, 0x30000, {}, NoFault},
            {0x0, 0x0, 0x0, {}, dummyFault1},
            {0x0, 0x8000, 0x40000, {}, NoFault},
            {0x0, 0x8000, 0x50000, {}, NoFault}
    });

    RangeList range_list;
    for (const auto &range: gen)
        range_list.emplace_back(range);

    // What the generator should return.
    const RangeList expected_gen{
        {0x10000, 0x8000, 0x30000, {}, NoFault},
        {0x18000, 0x0, 0x0, {}, dummyFault1},
        {0x18000, 0x8000, 0x40000, {}, NoFault},
        {0x20000, 0x8000, 0x50000, {}, NoFault}
    };
    EXPECT_THAT(range_list, Pointwise(GenRangeEq(), expected_gen));

    // What the generator should have been asked to translate.
    const RangeList expected_trans{
        {0x10000, 0x18000, 0x0, {}, NoFault},
        {0x18000, 0x10000, 0x0, {}, NoFault},
        {0x18000, 0x10000, 0x0, {}, NoFault},
        {0x20000, 0x8000, 0x0, {}, NoFault}
    };
    EXPECT_THAT(gen.args, Pointwise(TransRangeEq(), expected_trans));
}

TEST(TranslationGen, VariablePageSize)
{
    TestTranslationGen gen(0x10000, 0x20000, {
            // Results for translate.
            {0x0, 0x8000, 0x30000, {}, NoFault},
            {0x0, 0x10000, 0x40000, {}, NoFault},
            {0x0, 0x8000, 0x50000, {}, NoFault}
    });

    RangeList range_list;
    for (const auto &range: gen)
        range_list.emplace_back(range);

    // What the generator should return.
    const RangeList expected_gen{
        {0x10000, 0x8000, 0x30000, {}, NoFault},
        {0x18000, 0x10000, 0x40000, {}, NoFault},
        {0x28000, 0x8000, 0x50000, {}, NoFault}
    };
    EXPECT_THAT(range_list, Pointwise(GenRangeEq(), expected_gen));

    // What the generator should have been asked to translate.
    const RangeList expected_trans{
        {0x10000, 0x20000, 0x0, {}, NoFault},
        {0x18000, 0x18000, 0x0, {}, NoFault},
        {0x28000, 0x8000, 0x0, {}, NoFault}
    };
    EXPECT_THAT(gen.args, Pointwise(TransRangeEq(), expected_trans));
}

TEST(TranslationGenDeathTest, IncrementEndIterator)
{
    TestTranslationGen gen(0x10000, 0x20000);
    gtestLogOutput.str("");
    ASSERT_ANY_THROW(gen.end()++);
    EXPECT_THAT(gtestLogOutput.str(),
            HasSubstr("Can't increment end iterator."));
}
