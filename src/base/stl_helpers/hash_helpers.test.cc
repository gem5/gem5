/*
 * Copyright (c) 2023 Arteris, Inc. and its applicable licensors and
 * affiliates.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/stl_helpers/hash_helpers.hh"

using namespace gem5;

TEST(HashHelpers, isHashEnabled)
{
    EXPECT_TRUE(stl_helpers::is_hash_enabled<int>);
    EXPECT_TRUE(stl_helpers::is_hash_enabled<long>);
    EXPECT_TRUE(stl_helpers::is_hash_enabled<double>);
    EXPECT_TRUE(stl_helpers::is_hash_enabled<std::string>);
    EXPECT_TRUE(stl_helpers::is_hash_enabled<void*>);
    using vector_t = std::vector<int>;
    EXPECT_TRUE(stl_helpers::is_hash_enabled<vector_t>);
    using tuple_t = std::tuple<int, bool, int**, std::string(*)(float)>;
    EXPECT_TRUE(stl_helpers::is_hash_enabled<tuple_t>);
    EXPECT_TRUE((stl_helpers::is_hash_enabled<std::pair<vector_t, tuple_t>>));
    EXPECT_TRUE((stl_helpers::is_hash_enabled<
        std::unordered_map<tuple_t, vector_t>>));
}

// The following tests do not test the hash value as it is considered an
// implementation detail and there is no contract on the way that value is
// computed. Testing for hash quality is extremelly computationnaly intensive
// and is not suitable for unit tests. Consider these tests to be more of a
// "does it compile?" check as well as a small set of examples for the user.
TEST(HashHelpers, hashPair)
{
    auto p = std::make_pair(1, std::string("hello"));
    auto hashVal = stl_helpers::hash_value(p);
    auto hashFunc = stl_helpers::hash<std::pair<int, std::string>>{};
    EXPECT_EQ(hashVal, hashFunc(p));
}

TEST(HashHelpers, hashTuple)
{
    auto t = std::make_tuple(1, "hello", 4.2, std::make_pair(true, 0.f));
    auto hashVal = stl_helpers::hash_value(t);
    auto hashFunc = stl_helpers::hash<decltype(t)>{};
    EXPECT_EQ(hashVal, hashFunc(t));
}

TEST(HashHelpers, hashVector)
{
    auto v = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9};
    auto hashVal = stl_helpers::hash_value(v);
    auto hashFunc = stl_helpers::hash<decltype(v)>{};
    EXPECT_EQ(hashVal, hashFunc(v));
}
