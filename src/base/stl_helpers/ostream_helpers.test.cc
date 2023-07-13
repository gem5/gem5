/*
 * Copyright (c) 2023 Arteris, Inc. and its applicable licensors and
 * affiliates.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer; redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials
 * provided with the distribution; neither the name of the copyright holders
 * nor the names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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

#include <map>
#include <sstream>
#include <string_view>
#include <vector>

#include "base/stl_helpers/ostream_helpers.hh"


TEST(OstreamHelpers, pair) {
    using gem5::stl_helpers::operator<<;
    auto p = std::make_pair(1, 2);
    std::ostringstream os;
    os << p;
    EXPECT_EQ(os.str(), "(1, 2)");
}

TEST(OstreamHelpers, tuple) {
    using gem5::stl_helpers::operator<<;
    auto t = std::make_tuple(true,
        std::make_pair("Hello", std::string_view("World")), '!');
    std::ostringstream os;
    os << t;
    EXPECT_EQ(os.str(), "(1, (Hello, World), !)");
}

TEST(OstreamHelpers, vector) {
    using gem5::stl_helpers::operator<<;
    auto v = std::vector<const char*>{"abc", "defg", "hijklm", "\n"};
    std::ostringstream os;
    os << v;
    EXPECT_EQ(os.str(), "[ abc, defg, hijklm, \n, ]");
}

TEST(OstreamHelpers, map) {
    using gem5::stl_helpers::operator<<;
    auto m = std::map<char, int>{{'a', 0}, {'b', 1}, {'c', 2}, {'d', 3}};
    std::ostringstream os;
    os << m;
    EXPECT_EQ(os.str(), "[ (a, 0), (b, 1), (c, 2), (d, 3), ]");
}

TEST(OstreamHelpers, optional) {
    using gem5::stl_helpers::operator<<;
    auto m = std::make_optional<int>(42);
    std::ostringstream os;
    os << m;
    EXPECT_EQ(os.str(), "42");
    os.str("");
    m.reset();
    os << m;
    EXPECT_EQ(os.str(), "(-)");
}
