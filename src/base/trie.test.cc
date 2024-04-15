/*
 * Copyright (c) 2012 Google
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

#include <gtest/gtest.h>

#include <iostream>
#include <sstream>
#include <string>

#include "base/trie.hh"
#include "base/types.hh"

using namespace gem5;

namespace
{

static inline uint32_t *
ptr(uintptr_t val)
{
    return (uint32_t *)val;
}

} // anonymous namespace

class TrieTestData : public testing::Test
{
  protected:
    typedef Trie<Addr, uint32_t> TrieType;
    TrieType trie;

    std::string
    dumpTrie()
    {
        std::stringstream ss;
        trie.dump("test trie", ss);
        return ss.str();
    }
};

TEST_F(TrieTestData, Empty)
{
    EXPECT_EQ(trie.lookup(0x123456701234567), nullptr) << dumpTrie();
}

TEST_F(TrieTestData, SingleEntry)
{
    trie.insert(0x0123456789abcdef, 40, ptr(1));
    EXPECT_EQ(trie.lookup(0x123456701234567), nullptr) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x123456789ab0000), ptr(1)) << dumpTrie();
}

TEST_F(TrieTestData, TwoOverlappingEntries)
{
    trie.insert(0x0123456789abcdef, 40, ptr(1));
    trie.insert(0x0123456789abcdef, 36, ptr(2));
    EXPECT_EQ(trie.lookup(0x123456700000000), nullptr) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x123456789ab0000), ptr(2)) << dumpTrie();
}

TEST_F(TrieTestData, TwoOverlappingEntriesReversed)
{
    trie.insert(0x0123456789abcdef, 36, ptr(2));
    trie.insert(0x0123456789abcdef, 40, ptr(1));
    EXPECT_EQ(trie.lookup(0x123456700000000), nullptr) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x123456789ab0000), ptr(2)) << dumpTrie();
}

TEST_F(TrieTestData, TwoIndependentEntries)
{
    trie.insert(0x0123456789abcdef, 40, ptr(2));
    trie.insert(0x0123456776543210, 40, ptr(1));
    EXPECT_EQ(trie.lookup(0x0123456789000000), ptr(2)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456776000000), ptr(1)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456700000000), nullptr) << dumpTrie();
}

TEST_F(TrieTestData, TwoEntries)
{
    trie.insert(0x0123456789000000, 40, ptr(4));
    trie.insert(0x0123000000000000, 40, ptr(1));
    trie.insert(0x0123456780000000, 40, ptr(3));
    trie.insert(0x0123456700000000, 40, ptr(2));

    EXPECT_EQ(trie.lookup(0x0123000000000000), ptr(1)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456700000000), ptr(2)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456780000000), ptr(3)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456789000000), ptr(4)) << dumpTrie();
}

TEST_F(TrieTestData, RemovingEntries)
{
    TrieType::Handle node1, node2;
    trie.insert(0x0123456789000000, 40, ptr(4));
    trie.insert(0x0123000000000000, 40, ptr(1));
    trie.insert(0x0123456780000000, 40, ptr(3));
    node1 = trie.insert(0x0123456700000000, 40, ptr(2));
    node2 = trie.insert(0x0123456700000000, 32, ptr(10));

    EXPECT_EQ(trie.lookup(0x0123000000000000), ptr(1)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456700000000), ptr(10)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456780000000), ptr(10)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456789000000), ptr(10)) << dumpTrie();

    trie.remove(node2);

    EXPECT_EQ(trie.lookup(0x0123000000000000), ptr(1)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456700000000), ptr(2)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456780000000), ptr(3)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456789000000), ptr(4)) << dumpTrie();

    trie.remove(node1);

    EXPECT_EQ(trie.lookup(0x0123000000000000), ptr(1)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456700000000), nullptr) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456780000000), ptr(3)) << dumpTrie();
    EXPECT_EQ(trie.lookup(0x0123456789000000), ptr(4)) << dumpTrie();
}
